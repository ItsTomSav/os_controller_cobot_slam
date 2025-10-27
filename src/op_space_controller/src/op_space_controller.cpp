#include "op_space_controller/op_space_controller.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"  // Per HW_IF_POSITION, ecc.
#include "controller_interface/helpers.hpp"

//#include "geometry_msgs/msg/transform_stamped.hpp"
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include <tf2_eigen/tf2_eigen.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

namespace op_space_controller
{

OpSpaceController::OpSpaceController()
: controller_interface::ControllerInterface(),
  pinocchio_model_loaded_(false),
  trajectory_active_(false),
  current_trajectory_index_(0)
{
  // CONSTRUCTOR
}

// ########################### CONTROLLER INITIALIZATION - on_init()  ###########################

controller_interface::CallbackReturn OpSpaceController::on_init()
{
  // Declare parameters
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  auto_declare<std::string>("command_interface_type", "effort"); // Default a "effort"

    // Leggi immediatamente i parametri necessari per la configurazione delle interfacce e memorizzali nei membri della classe.
    // Nota: qui usiamo direttamente this->get_node() perché siamo in un metodo del controller
  try {
      joint_names_ = get_node()->get_parameter("joints").as_string_array();
      command_interface_type_ = get_node()->get_parameter("command_interface_type").as_string();
      num_joints_ = joint_names_.size();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parametro 'joints' o 'command_interface_type' non dichiarato in YAML ma richiesto da on_init: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Tipo parametro errato per 'joints' o 'command_interface_type' in on_init: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
  }

  if (joint_names_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parametro 'joints' è vuoto in on_init. Impossibile procedere.");
      return controller_interface::CallbackReturn::ERROR;
  }


  // Declare other parameters of controller
  auto_declare<std::string>("urdf_file_path", "");
  auto_declare<std::string>("base_link_name", "base_link");
  auto_declare<std::string>("ee_link_name", "tool_link"); // Assicurati che il default sia sensato
  auto_declare<std::string>("map_frame_name", "map");
  auto_declare<std::string>("trajectory_file_path", "");
  auto_declare<double>("trajectory_sample_dt", 0.1);
  auto_declare<double>("kp_linear", 50.0);
  auto_declare<double>("kd_linear", 10.0);
  auto_declare<double>("kp_angular", 25.0);
  auto_declare<double>("kd_angular", 5.0);
  
  RCLCPP_INFO(get_node()->get_logger(), "Controller '%s' on_init: parametri dichiarati.", get_node()->get_name());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OpSpaceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto & name : joint_names_)
    conf.names.push_back(name + "/" + command_interface_type_);
  return conf;
}

controller_interface::InterfaceConfiguration OpSpaceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto & name : joint_names_)
  {
    conf.names.push_back(name + "/position");
    conf.names.push_back(name + "/velocity");
  }
  return conf;
}

// ########################### CONTROLLER CONFIGURATION - on_configure()  ###########################

controller_interface::CallbackReturn OpSpaceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();
  RCLCPP_INFO(get_node()->get_logger(), "Configuring OpSpaceController...");

  // 1. Read parameters declared in on_init()
  try {
    //joint_names_ = get_node()->get_parameter("joints").as_string_array();  // Questo è già fatto in on_init()
    //command_interface_type_ = get_node()->get_parameter("command_interface_type").as_string();  
    urdf_file_path_ = get_node()->get_parameter("urdf_file_path").as_string();
    base_link_name_ = get_node()->get_parameter("base_link_name").as_string();
    ee_link_name_ = get_node()->get_parameter("ee_link_name").as_string();
    map_frame_name_ = get_node()->get_parameter("map_frame_name").as_string();

    trajectory_file_path_ = get_node()->get_parameter("trajectory_file_path").as_string();
    trajectory_sample_dt_ = get_node()->get_parameter("trajectory_sample_dt").as_double(); // Questo valore serve a read_trajectory_from_file

    // Read gains declared in on_init()
    kp_linear_ = get_node()->get_parameter("kp_linear").as_double();
    kd_linear_ = get_node()->get_parameter("kd_linear").as_double();
    kp_angular_ = get_node()->get_parameter("kp_angular").as_double();
    kd_angular_ = get_node()->get_parameter("kd_angular").as_double();

  } catch (const std::exception &e) {
    RCLCPP_FATAL(logger, "Error reading parameters in on_configure: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
/*   if (joint_names_.empty())   //Già fatto in on_init()
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Set num_joints_ here
  num_joints_ = joint_names_.size();
  RCLCPP_INFO(logger, "Controller configurato per %zu giunti.", num_joints_);
  for(const auto& name : joint_names_) { RCLCPP_DEBUG(logger, "- Giunto: %s", name.c_str()); } */
  

  // 2. Load URDF into Pinocchio
  if (urdf_file_path_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'urdf_file_path' is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  try{
    pinocchio::urdf::buildModel(urdf_file_path_, model_); // model_ is member pinocchio::Model

    // GRAVITY VECTOR PINOCCHIO -- AGGIUNTO4
    // Imposta esplicitamente il vettore di gravità nel modello di Pinocchio. La convenzione ROS/URDF ha l'asse Z che punta verso l'alto.
    model_.gravity.linear(Eigen::Vector3d(0, 0, -9.81));

    //data_ = pinocchio::Data(model_);    //data_ = std::make_shared<pinocchio::Data>(model_);    // data_ is std::shared_ptr<pinocchio::Data>
    data_ = std::make_shared<pinocchio::Data>(model_);

    if (!model_.existFrame(ee_link_name_)) { // Usa il membro ee_link_name_ letto dai parametri
      RCLCPP_ERROR(logger, "Frame End-Effector '%s' non trovato nel modello Pinocchio!", ee_link_name_.c_str());
      pinocchio_model_loaded_ = false; // Importante resettare se fallisce qui
      return controller_interface::CallbackReturn::ERROR;
    }

    // ee_frame_id_pin_ non è dichiarato come membro nel tuo ultimo .hpp, ma sarebbe utile:
    ee_frame_id_pin_ = model_.getFrameId(ee_link_name_);

    pinocchio_model_loaded_ = true;
    RCLCPP_INFO(logger, "Modello Pinocchio '%s' (nv=%ld, nq=%ld) caricato da URDF: %s. End-Effector Link: '%s'", model_.name.c_str(), static_cast<long>(model_.nv), static_cast<long>(model_.nq), urdf_file_path_.c_str(), ee_link_name_.c_str());
  } catch (const std::exception & e){
    RCLCPP_FATAL(logger, "Failed to build Pinocchio model from '%s': %s", urdf_file_path_.c_str(), e.what());
    pinocchio_model_loaded_ = false;
    return controller_interface::CallbackReturn::ERROR;
  }

  // 3. Setup TF buffer and listener
  try {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, get_node(), false); // false per non avviare un thread dedicato se non strettamente necessario
    RCLCPP_INFO(logger, "TF Buffer and Listener initialized.");
  } catch (const std::exception &e) {
    RCLCPP_FATAL(logger, "TF initialization error: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // --- 4. Read Trajectory File ---
  // L'inizializzazione di trajectory_active_, start_time_, current_trajectory_index_ è stata spostata in on_activate() per un timing più corretto.
  // Qui carichiamo solo i dati in desired_trajectory_.
  if (!trajectory_file_path_.empty()) {
      if (!read_trajectory_from_file(trajectory_file_path_)) {
          RCLCPP_ERROR(logger, "Error loading trajectory from file: %s. The controller may not have a trajectory to follow.", trajectory_file_path_.c_str());
          // Decidi se questo è un errore fatale. Se il controller può operare senza una traiettoria
          // precaricata (es. aspettandone una da un topic), allora potrebbe essere solo un WARNING.
        return controller_interface::CallbackReturn::ERROR; // Decommenta se una traiettoria è obbligatoria
      } else {
          RCLCPP_INFO(logger, "Traiettoria caricata (%zu waypoint) da %s.", desired_trajectory_.size(), trajectory_file_path_.c_str());
      }
  } else {
    RCLCPP_INFO(logger, "No trajectory file ('trajectory_file_path') specified. The controller will start without a preloaded trajectory..");
  }
  // Read trajectory file - VECCHIO
  /* if (!read_trajectory_from_file(trajectory_file_path_)){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read trajectory file %s", trajectory_file_path_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  trajectory_active_ = true;
  start_time_ = get_node()->now();
  current_trajectory_index_ = 0; */


  // 5. Initialize/Resize Eigen Vectors
  // model_.nv è il numero di gradi di libertà nello spazio delle velocità (tangente)
  // model_.nq è il numero di gradi di libertà nello spazio delle configurazioni (posizioni)
  // Per manipolatori seriali senza giunti flottanti, nq >= nv, e nv corrisponde a num_joints_
  if (pinocchio_model_loaded_) {
      q_ = Eigen::VectorXd::Zero(model_.nq); // Usa nq per le posizioni
      dq_ = Eigen::VectorXd::Zero(model_.nv); // Usa nv per le velocità
      tau_ = Eigen::VectorXd::Zero(model_.nv); // Le coppie agiscono sullo spazio delle velocità
      RCLCPP_INFO(logger, "Eigen vectors q (size %ld), dq (size %ld), tau (size %ld) initializated.", static_cast<long>(model_.nq), static_cast<long>(model_.nv), static_cast<long>(model_.nv) );
  } else {
    // Questo non dovrebbe accadere se i check precedenti sono corretti
    RCLCPP_ERROR(logger, "Modello Pinocchio non caricato, impossibile inizializzare vettori Eigen q, dq, tau.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // RIMOSSA la chiamata a wait_for_transform da on_configure.
  // La disponibilità della TF verrà controllata nel loop update() o in on_activate()
  // prima di iniziare il controllo attivo.
  // Wait for transform map -> ee_link
  //if (!wait_for_transform(map_frame_name_, ee_link_name_, rclcpp::Duration::from_seconds(5.0)))
  //  return controller_interface::CallbackReturn::ERROR;


  //AGGIUNTO4 FINE on_configure
  RCLCPP_INFO(logger, "Ordine da Parametri ROS ('joints'):");
  for (const auto & name : joint_names_) {
      RCLCPP_INFO(logger, "- %s", name.c_str());
  }

  RCLCPP_INFO(logger, "Ordine Interno del Modello Pinocchio:");
  // model_.names[0] è 'universe', quindi partiamo da 1
  for (size_t i = 1; i < model_.names.size(); ++i) {
      RCLCPP_INFO(logger, "- %s", model_.names[i].c_str());
  }

  // Confronto automatico
  bool order_is_correct = true;
  if (joint_names_.size() != static_cast<size_t>(model_.njoints - 1)) { // njoints include 'universe'
      order_is_correct = false;
  } else {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
          if (joint_names_[i] != model_.names[i+1]) {
              order_is_correct = false;
              break;
          }
      }
  }

  if (order_is_correct) {
      RCLCPP_INFO(logger, "VERIFICA OK: L'ordine dei giunti ROS e Pinocchio corrisponde.");
  } else {
      RCLCPP_FATAL(logger, "ERRORE CRITICO: L'ordine dei giunti ROS e Pinocchio NON corrisponde! Questo causerà calcoli dinamici errati.");
  }

  RCLCPP_INFO(logger, "Controller '%s' configurato con successo.", get_node()->get_name());
  return controller_interface::CallbackReturn::SUCCESS;
}

// Funzione per leggere la traiettoria da un file chiamata in on_configure()

// Assicurati che la tua funzione read_trajectory_from_file sia definita come membro e usi trajectory_sample_dt_ (membro della classe)
bool OpSpaceController::read_trajectory_from_file(const std::string & file_path) {
  // trajectory_sample_dt_ dovrebbe essere stato letto dai parametri e memorizzato come membro
  // Se non è un membro, passalo come argomento o leggilo qui:
  // double dt_param = get_node()->get_parameter("trajectory_sample_dt").as_double();

  std::ifstream file_stream(file_path);
  if (!file_stream.is_open()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to open trajectory file: %s", file_path.c_str());
      return false;
  }
  //desired_trajectory_.clear();   // aggiornato come sotto
  this->desired_trajectory_.clear(); // Opera sul membro della classe

  std::string line;
  bool header_skipped = false; 
  double current_time_from_start_sec = 0.0;

  while (std::getline(file_stream, line)) {
      if (line.empty() || (line[0] == '#' && !header_skipped) ) { // Salta solo la prima riga di commento come header
          header_skipped = true;
          continue;
      }
      if (line[0] == '#') continue; // Salta altre righe di commento

      std::istringstream iss(line);
      double x, y, z, qx, qy, qz, qw;
      if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw)) {
          RCLCPP_WARN(get_node()->get_logger(), "Malformed line in trajectory file (7 values ​​expected): %s", line.c_str());
          continue; 
        }
      EEWaypoint waypoint; // Usa la struct EEWaypoint (presente in hpp)
      // Ricostruisci time_from_start usando il membro trajectory_sample_dt_
      waypoint.time_from_start = rclcpp::Duration::from_seconds(current_time_from_start_sec);
      Eigen::Vector3d pos(x, y, z);  //prima c'era position al posto di pos
      Eigen::Quaterniond quat(qw, qx, qy, qz);  //prima c'era orientation al posto di quat
      quat.normalize();
      waypoint.pose_desired = pinocchio::SE3(quat.toRotationMatrix(), pos);  // velocity and acc not in file? set zero or parse
      waypoint.velocity_desired = pinocchio::Motion::Zero();
      waypoint.acceleration_desired = pinocchio::Motion::Zero();
      this->desired_trajectory_.push_back(waypoint); // Aggiungi al membro della classe
      current_time_from_start_sec += this->trajectory_sample_dt_; // Usa il membro della classe
  }
  file_stream.close();

  if (desired_trajectory_.empty()) {
    if (header_skipped) { // C'era un header ma nessun dato
          RCLCPP_ERROR(get_node()->get_logger(), "No valid waypoint loaded by: %s (after the header).", file_path.c_str());
    } else if (!file_path.empty()) { // Il file era specificato ma completamente vuoto o illeggibile
          RCLCPP_ERROR(get_node()->get_logger(), "Empty or invalid trajectory file: %s", file_path.c_str());
    } else { // Nessun file specificato, nessun waypoint caricato (OK)
        RCLCPP_INFO(get_node()->get_logger(), "No trajectory file specified, no waypoint loaded.");
        return true; // Non è un errore se nessun file è stato specificato e trajectory_file_path_ è vuoto
    }
    return false; // Errore se il file era specificato ma vuoto o malformato
  }
    
  RCLCPP_INFO(get_node()->get_logger(), "Loaded %zu waypoints from: %s. Estimated trajectory duration: %.3f s", 
            desired_trajectory_.size(), file_path.c_str(), 
            desired_trajectory_.empty() ? 0.0 : desired_trajectory_.back().time_from_start.seconds());
  return true;
}

//Funzione per leggere la traiettoria da un file -- VECCHIA --
/* bool OpSpaceController::read_trajectory_from_file(const std::string & file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) return false;
  desired_trajectory_.clear();
  std::string line;
  while (std::getline(file, line))
  {
    std::stringstream ss(line);
    double x,y,z,qx,qy,qz,qw,t;
    ss >> x >> y >> z >> qx >> qy >> qz >> qw >> t;
    EEWaypoint wp;
    wp.pose = pinocchio::SE3(Eigen::Quaterniond(qw,qx,qy,qz).toRotationMatrix(), Eigen::Vector3d(x,y,z));
    // velocity and acc not in file? set zero or parse
    wp.velocity = pinocchio::Motion::Zero();
    wp.acceleration = pinocchio::Motion::Zero();
    wp.time_from_start = rclcpp::Duration::from_seconds(t);
    desired_trajectory_.push_back(wp);
  }
  return true;
} */


// ########################### ACTIVATING CONTROLLER - on_activate()  ###########################

controller_interface::CallbackReturn OpSpaceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(get_node()->get_logger(), "Activating controller '%s'...", get_node()->get_name());

  // 0. Pulisci le handles da attivazioni precedenti
  state_if_position_handles_.clear();
  state_if_velocity_handles_.clear();
  cmd_if_effort_handles_.clear();

  // 1. Verifica che i parametri necessari (come joint_names_) siano stati caricati in on_configure
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' list is empty. The controller is not configured correctly.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!pinocchio_model_loaded_) { // Assumendo che pinocchio_model_loaded_ sia impostato in on_configure
    RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio model not loaded. Cannot activate.");
    return controller_interface::CallbackReturn::ERROR;
  }
  // num_joints_ dovrebbe essere già stato impostato in on_configure da joint_names_.size()


  // 2. Ottieni e ordina le interfacce di stato per la POSIZIONE
  // state_interfaces_ è un membro protetto ereditato da ControllerInterface
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION, state_if_position_handles_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failure to get or order state interfaces for POSITION.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (state_if_position_handles_.size() != num_joints_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Wrong number of position handles obtained: %zu, expected: %zu", state_if_position_handles_.size(), num_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Got %zu state interfaces for the position.", state_if_position_handles_.size());

  // 3. Ottieni e ordina le interfacce di stato per la VELOCITÀ
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names_, hardware_interface::HW_IF_VELOCITY, state_if_velocity_handles_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failure to get or order state interfaces for VELOCITY.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (state_if_velocity_handles_.size() != num_joints_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Wrong number of velocity handles obtained: %zu, expected: %zu", state_if_velocity_handles_.size(), num_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Got %zu state interfaces for the velocity.", state_if_velocity_handles_.size());

  // 4. Ottieni e ordina le interfacce di comando
  //    command_interface_type_ è un membro stringa letto dai parametri in on_configure (es. "effort")
  //    command_interfaces_ è un membro protetto ereditato
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names_, command_interface_type_, cmd_if_effort_handles_)) // Usa il nome corretto del tuo vettore membro
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Fallimento nell'ottenere o ordinare le interfacce di comando di tipo '%s'.", command_interface_type_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (cmd_if_effort_handles_.size() != num_joints_) { // Usa il nome corretto del tuo vettore membro
    RCLCPP_ERROR(get_node()->get_logger(), "Numero errato di handle di comando '%s' ottenuti: %zu, attesi: %zu", command_interface_type_.c_str(), cmd_if_effort_handles_.size(), num_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Ottenute %zu interfacce di comando di tipo '%s'.", cmd_if_effort_handles_.size(), command_interface_type_.c_str());


  // 5. Inizializza i vettori Eigen per lo stato e il comando (se non già fatto in on_configure)
  //q_ = Eigen::VectorXd::Zero(model_.nq);     // Cambio da q_ros_ a q_
  //dq_ = Eigen::VectorXd::Zero(model_.nv);    // Cambio da v_ros_ a dq_
  //tau_ = Eigen::VectorXd::Zero(model_.nv);   // Cambio da tau_command_pin_ a tau_
  tau_.setZero(); // Usa il membro tau_ e azzera il comando iniziale.
  RCLCPP_INFO(get_node()->get_logger(), "Membro tau_ azzerato.");

  // 6. Leggi lo stato iniziale dei joint per popolare q_ros_, v_ros_
  // Questo è importante per avere un q_attuale e v_attuale corretti al primo ciclo di update()
  try {
    for (size_t i = 0; i < num_joints_; ++i) {
        q_(i) = state_if_position_handles_[i].get().get_value();
        dq_(i) = state_if_velocity_handles_[i].get().get_value();
    }
    RCLCPP_INFO(get_node()->get_logger(), "Stato iniziale dei joint letto e memorizzato.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Eccezione durante la lettura dello stato iniziale dei joint: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
    
  // 7. Inizializza/Resetta lo stato per l'inseguimento della traiettoria
  current_trajectory_index_ = 0;
  if (!desired_trajectory_.empty()) { // desired_trajectory_ dovrebbe essere stata caricata in on_configure
    trajectory_start_time_ = get_node()->now(); // Usa il tempo corrente del nodo come inizio
    trajectory_active_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Inizio inseguimento traiettoria precaricata con %zu punti.", desired_trajectory_.size());
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Nessuna traiettoria precaricata. Il controller attenderà comandi o rimarrà inattivo.");
    trajectory_active_ = false;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller '%s' attivato con successo.", get_node()->get_name());

  return controller_interface::CallbackReturn::SUCCESS;
 


    

  // VECCHIO
  /*   // Acquire state and command interfaces
    auto state_interfaces = get_state_interface_handles();
    auto command_interfaces = get_command_interface_handles();
    
    if (state_interfaces.size() != joint_names_.size()*2 || command_interfaces.size() != joint_names_.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unexpected number of interfaces");
      return controller_interface::CallbackReturn::ERROR;
    }
    
    // Loan interfaces
    joint_state_pos_.clear(); joint_state_vel_.clear(); joint_command_effort_.clear();
    for (size_t i=0; i<joint_names_.size(); ++i)
    {
      joint_state_pos_.push_back(state_interfaces[2*i]);
      joint_state_vel_.push_back(state_interfaces[2*i+1]);
      joint_command_effort_.push_back(command_interfaces[i]);
    }

    q_.setZero(model_.nq);
    dq_.setZero(model_.nv);
    tau_.setZero(model_.nv); */
}


// ########################### DEACTIVATING CONTROLLER - on_deactivate()  ###########################

controller_interface::CallbackReturn OpSpaceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating controller '%s'...", get_node()->get_name());
  trajectory_active_ = false; // Interrompi l'inseguimento attivo della traiettoria
  
  // Invia comandi sicuri (es. sforzi zero) ai giunti.
  // È importante farlo prima di svuotare gli handle, finché sono validi.
  if (!cmd_if_effort_handles_.empty()) { // Usa il nome corretto del tuo vettore membro
      if (cmd_if_effort_handles_.size() == num_joints_) {
          for (size_t i = 0; i < num_joints_; ++i) {
              try {
                  cmd_if_effort_handles_[i].get().set_value(0.0);
              } catch (const std::bad_function_call& e) {
                  RCLCPP_ERROR(get_node()->get_logger(), "Exception during set_value(0.0) in on_deactivate for joint %zu: %s. The handle may no longer be valid.", i, e.what());
                  // Questo potrebbe succedere se le interfacce sono già state rilasciate o non sono valide.
              }
            }
            RCLCPP_INFO(get_node()->get_logger(), "Zero effort commands sent to joints.");
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "Number of command handles does not match num_joints_. Cannot send safe commands.");
        }
  } else {
      RCLCPP_WARN(get_node()->get_logger(), "Command handles vector is empty. Cannot send safe commands.");
  }

  // Svuota i vettori di reference_wrapper. Questo non "rilascia" le interfacce dal controller_manager,
  // ma pulisce i tuoi riferimenti interni. Il controller_manager gestisce il claim/release
  // effettivo delle interfacce quando il controller cambia stato.
  state_if_position_handles_.clear();
  state_if_velocity_handles_.clear();
  cmd_if_effort_handles_.clear(); // Usa il nome corretto del tuo vettore membro
    
  RCLCPP_INFO(get_node()->get_logger(), "Hardware interface references emptied.");
  RCLCPP_INFO(get_node()->get_logger(), "Controller '%s' deactivated.", get_node()->get_name());

  return controller_interface::CallbackReturn::SUCCESS;
}


 // ########################### CLEANING UP CONTROLLER - on_cleanup()  ###########################

controller_interface::CallbackReturn OpSpaceController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Cleaning up controller '%s'...", get_node()->get_name());
  desired_trajectory_.clear();
  trajectory_active_ = false;

  // Resetta Pinocchio
  if (pinocchio_model_loaded_) {
    data_.reset(); // Rilascia prima i dati che dipendono dal modello
    // model_ = pinocchio::Model();
    pinocchio_model_loaded_ = false;
  }

  // Pulisci gli handle delle interfacce
  state_if_position_handles_.clear();
  state_if_velocity_handles_.clear();
  cmd_if_effort_handles_.clear(); // O il nome del tuo vettore di command handles

  RCLCPP_INFO(get_node()->get_logger(), "Controller '%s' cleaned up.", get_node()->get_name());

  return controller_interface::CallbackReturn::SUCCESS;
}


// ########################### ERROR CONTROLLER HANDLING - on_error()  ###########################

controller_interface::CallbackReturn OpSpaceController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_node()->get_logger(), "Controller '%s' encountered an error. Attempting to stop robot.", get_node()->get_name());
  
  trajectory_active_ = false; // Ferma l'inseguimento

  // Invia comandi sicuri (es. sforzi zero)
  if (!cmd_if_effort_handles_.empty() && cmd_if_effort_handles_.size() == num_joints_) {
    for (size_t i = 0; i < num_joints_; ++i) {
      try {
        cmd_if_effort_handles_[i].get().set_value(0.0);
      } catch (const std::exception& e) {
        // Non fare molto altro qui per evitare ulteriori errori in cascata
        RCLCPP_ERROR(get_node()->get_logger(), "Sub-exception in on_error while setting zero effort for joint %zu: %s", i, e.what());
      }
    }
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Cannot send safe stop commands in on_error: command handles not ready/mismatched.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}



// ########################### UPDATE LOOP - update()  ###########################

controller_interface::return_type OpSpaceController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  /*
  // INIZIO PROVA 1.1 -----------
  if (!pinocchio_model_loaded_) { return controller_interface::return_type::ERROR; }

  // 1. Leggi lo stato attuale dei giunti q_ e dq_
  for (size_t i = 0; i < num_joints_; ++i) {
    q_(i) = state_if_position_handles_[i].get().get_value();
    dq_(i) = state_if_velocity_handles_[i].get().get_value();
  }
  pinocchio::rnea(model_, *data_, q_, dq_, Eigen::VectorXd::Zero(model_.nv));
  Eigen::VectorXd nle_torques = data_->tau;

  //double kd_damping_gain = 5.0;
  Eigen::Matrix<double, 6, 1> kd_joints; // Vettore colonna 6x1
  kd_joints << 2.0,  // Guadagno per joint_1
               2.0,  // Guadagno per joint_2
               2.0,  // Guadagno per joint_3
               0.5,  // Guadagno per joint_4 (polso)
               0.5,  // Guadagno per joint_5 (polso)
               0.5;  // Guadagno per joint_6 (polso)
  Eigen::VectorXd damping_torques = -(kd_joints.asDiagonal() * dq_);
  tau_ = nle_torques + damping_torques;

  for (size_t i = 0; i < num_joints_; ++i) {
    cmd_if_effort_handles_[i].get().set_value(tau_(i));
  }

  // Log per il debug
  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "\n--- Test Stabilizzazione ---\n"
    "dq_attuale:      [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n"
    "Coppie N(q,dq):  [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n"
    "Coppie Smorz.:   [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n"
    "Coppie Finali:   [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    dq_(0), dq_(1), dq_(2), dq_(3), dq_(4), dq_(5),
    nle_torques(0), nle_torques(1), nle_torques(2), nle_torques(3), nle_torques(4), nle_torques(5),
    damping_torques(0), damping_torques(1), damping_torques(2), damping_torques(3), damping_torques(4), damping_torques(5),
    tau_(0), tau_(1), tau_(2), tau_(3), tau_(4), tau_(5)
  ); */
  // FINE PROVA 1.1 -----------


  auto logger = get_node()->get_logger();
  const int THROTTLE_MS = 1000; // Logga ogni 1000ms (1 secondo)

  // --- A. CONTROLLI PRELIMINARI E VERIFICA STATO ---
  if (!pinocchio_model_loaded_) {
      RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000, "Modello Pinocchio non caricato. Update saltato.");
      // Invia comandi sicuri (sforzi nulli)
      for (size_t i = 0; i < cmd_if_effort_handles_.size(); ++i) { command_interfaces_[i].set_value(0.0); }
      return controller_interface::return_type::ERROR;
  }

  if (!trajectory_active_ || desired_trajectory_.empty()) {
      RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 2000, "Traiettoria terminata o non esistente.");
      // Invia comandi sicuri (sforzi nulli)
      //for (size_t i = 0; i < cmd_if_effort_handles_.size(); ++i) { command_interfaces_[i].set_value(0.0); }
      //return controller_interface::return_type::OK;
  }


  // --- B. LETTURA STATO ATTUALE DEL ROBOT ---

  // 1. Leggi Stato Attuale dei Joint (q_attuale, dq_attuale)
  // Assumiamo che l'ordine delle interfacce corrisponda a quello dei joint nel modello Pinocchio.
  if (state_if_position_handles_.size() != num_joints_ || state_if_velocity_handles_.size() != num_joints_) {
      RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000, "Numero errato di state handles in update.");
      return controller_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < num_joints_; ++i) {
      q_(i) = state_if_position_handles_[i].get().get_value();
      dq_(i) = state_if_velocity_handles_[i].get().get_value();
  }

  // LOGGING: Stato dei giunti -- AGGIUNTO
  RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), THROTTLE_MS, "Stato Attuale q : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", q_(0), q_(1), q_(2), q_(3), q_(4), q_(5));
  RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), THROTTLE_MS, "Stato Attuale dq: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", dq_(0), dq_(1), dq_(2), dq_(3), dq_(4), dq_(5));

  // 2. Ottieni Posa Attuale dell'End-Effector (X_attuale da TF/SLAM)
  Eigen::Isometry3d X_actual_map_eigen; // Posa EE nel frame 'map'
  geometry_msgs::msg::TransformStamped tf_map_to_ee;
  try {
      tf_map_to_ee = tf_buffer_->lookupTransform(
          map_frame_name_,         // Target frame (letto da param, es. "map")
          ee_link_name_,      // Source frame (letto da param, es. "tool_link")
          tf2::TimePointZero  // Ultima trasformazione disponibile
          // Considera un piccolo timeout se necessario, ma update() dovrebbe essere veloce
          // rclcpp::Duration::from_seconds(0.01) //Se voglio un timeout devo decommentare
      );
      X_actual_map_eigen = tf2::transformToEigen(tf_map_to_ee);
      //tf2::fromMsg(tf_map_to_ee.transform, X_actual_map_eigen);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000, "TF lookup da '%s' a '%s' fallito: %s. Sforzi nulli.",
                         map_frame_name_.c_str(), ee_link_name_.c_str(), ex.what());
      for (size_t i = 0; i < cmd_if_effort_handles_.size(); ++i) { command_interfaces_[i].set_value(0.0); }
      return controller_interface::return_type::ERROR; // O OK se vuoi che continui a provare
  }

  // LOGGING: Posa attuale EE -- AGGIUNTO
  Eigen::Vector3d pos_actual = X_actual_map_eigen.translation();
  Eigen::Quaterniond quat_actual(X_actual_map_eigen.rotation());
  //RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), THROTTLE_MS, "Posa Attuale EE (map): Pos=[%.3f, %.3f, %.3f]", pos_actual.x(), pos_actual.y(), pos_actual.z());
  RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 500, "Posa Attuale EE (map): Pos=[%.3f, %.3f, %.3f], Quat=[w:%.2f, x:%.2f, y:%.2f, z:%.2f]", pos_actual.x(), pos_actual.y(), pos_actual.z(), quat_actual.w(), quat_actual.x(), quat_actual.y(), quat_actual.z());


  // MODIFICA -- VARIAZIONE DI AGGIORNAMENTO WAYPOINT ------------------- AGGIUNTO5
  // --- C. DETERMINAZIONE DEL SETPOINT DESIDERATO DALLA TRAIETTORIA ---
 /* COMMENTATO PER METTERE "AGGIUNTA5"
  rclcpp::Duration time_in_trajectory = time - trajectory_start_time_;
  EEWaypoint target_waypoint; // Conterrà X_d, Xd_dot, Xd_ddot interpolati/selezionati

  // LOGGING: Tempo trascorso in traiettoria -- AGGIUNTO
  RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), THROTTLE_MS,"Tempo trascorso in traiettoria: %.3f s", time_in_trajectory.seconds());

  // Logica di selezione/interpolazione del waypoint (DA IMPLEMENTARE ROBUSTAMENTE)
  // Questo è un esempio base che seleziona il waypoint corrente o il prossimo.
  // TODO: Una vera interpolazione (es. SLERP per SE3, LERP per Motion) è necessaria per un inseguimento fluido.

  if (current_trajectory_index_ < desired_trajectory_.size()) {
      size_t next_idx = current_trajectory_index_ + 1;
      if (next_idx < desired_trajectory_.size() &&
          time_in_trajectory >= desired_trajectory_[next_idx].time_from_start) {
          current_trajectory_index_ = next_idx;
          // LOGGING: Avanzamento a un nuovo waypoint -- AGGIUNTO
          RCLCPP_INFO(get_node()->get_logger(), "Avanzamento a waypoint indice: %zu (tempo target: %.3f s)", current_trajectory_index_, desired_trajectory_[current_trajectory_index_].time_from_start.seconds());
      }
      target_waypoint = desired_trajectory_[current_trajectory_index_];
  } else {
      RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 2000, "Fine traiettoria raggiunta. Invio sforzi nulli.");
      trajectory_active_ = false; // Ferma il tracking attivo
      for (size_t i = 0; i < cmd_if_effort_handles_.size(); ++i) { command_interfaces_[i].set_value(0.0); }
      return controller_interface::return_type::OK;
  }
  */
   //AGGIUNTO5
   if (!trajectory_active_ || current_trajectory_index_ >= desired_trajectory_.size()) {
      // Se non c'è traiettoria o è finita, mantieni la posizione con smorzamento
      RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 2000, "SONO ENTRATO QUI, FACCIO QUESTO!");  //AGGIUNTA9 per capire se entro in questo if
      pinocchio::rnea(model_, *data_, q_, dq_, Eigen::VectorXd::Zero(model_.nv));
      double kd_joint_damping = 2.0;
      tau_ = data_->tau - kd_joint_damping * dq_;
      for (size_t i = 0; i < num_joints_; ++i) { cmd_if_effort_handles_[i].get().set_value(tau_(i)); }
      return controller_interface::return_type::OK;
  }

  // 2. Seleziona il waypoint target ATTUALE (non avanza in base al tempo)       //AGGIUNTO5
  //EEWaypoint& target_waypoint = desired_trajectory_[current_trajectory_index_];  //AGGIUNTO5

  //Sostituisco la riga di sopra con questa per imporre l'ultimo waypoint ciclico alla fine della traiettoria -- AGGIUNTO9
  //Inoltre aggiungo un if al controllo della norma dell'errore alla fine
  EEWaypoint& target_waypoint = (current_trajectory_index_ < desired_trajectory_.size())
                                ? desired_trajectory_[current_trajectory_index_]
                                : desired_trajectory_.back();

  const pinocchio::SE3& X_d_map = target_waypoint.pose_desired;                   // Posa desiderata nel frame map
  const pinocchio::Motion& Xd_dot_map = target_waypoint.velocity_desired;         // Velocità desiderata nel frame map
  const pinocchio::Motion& Xd_ddot_map = target_waypoint.acceleration_desired;    // Accelerazione desiderata nel frame map

  // LOGGING: Posa desiderata -- AGGIUNTO
  Eigen::Vector3d pos_desired = X_d_map.translation();
  RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), THROTTLE_MS, "Posa Desiderata EE (map): Pos=[%.3f, %.3f, %.3f]", pos_desired.x(), pos_desired.y(), pos_desired.z());


  // --- D. CALCOLI CINEMATICI E DINAMICI CON PINOCCHIO ---
  // È cruciale che q_ e dq_ siano aggiornati prima di questi calcoli.

  // 1. Calcola tutti i termini necessari: FK, Jacobiane, M, N, dJ
  //    computeAllTerms calcola M(q) in data_.M, N(q,dq) in data_.nle (C(q,dq)dq + G(q)),
  //    e prepara per il calcolo di J e dJ.
  pinocchio::computeAllTerms(model_, *data_, q_, dq_);
  // Assicura simmetria di M
  data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose().template triangularView<Eigen::StrictlyLower>();

  // Ottieni la posa dell'end-effector calcolata da Pinocchio (nel frame base di Pinocchio, di solito base_link)
  // ee_frame_id_pin_ è stato impostato in on_configure
  const pinocchio::SE3& X_actual_pin_base = data_->oMf[ee_frame_id_pin_];

  // Trasforma la posa desiderata X_d_map nel frame base di Pinocchio (se diverso da map)
  // Per ora, assumiamo che il tuo TF statico world->map e world->base_link renda map e base_link coincidenti
  // o che tu abbia una trasformazione map_to_base_pin_tf disponibile.
  // Se map e base_link (usato come frame 0 da Pinocchio) sono coincidenti:
  pinocchio::SE3 X_d_pin_base = X_d_map; // Semplificazione, DA VERIFICARE ATTENTAMENTE!
                                        // Se X_d_map è in 'map' e la base di Pinocchio è 'base_link',
                                        // e 'map' e 'base_link' non sono lo stesso frame, serve una trasformazione:
                                        // X_base_map = TF(base_link -> map)
                                        // X_d_pin_base = X_base_map * X_d_map;

  // 2. Calcola Jacobiana J e Derivata dJ (nel frame base di Pinocchio, per coerenza)
  Eigen::MatrixXd J_base(6, model_.nv);
  J_base.setZero();
  // LOCAL: Jacobiana le cui colonne sono i twists dei giunti proiettati sull'asse del giunto,
  // espressi nel frame locale del giunto.
  // WORLD: Jacobiana le cui colonne sono i twists dei giunti proiettati sull'asse del giunto,
  // espressi nel frame mondo (frame 0 di Pinocchio, di solito base_link).
  // LOCAL_WORLD_ALIGNED: Come WORLD, ma gli assi sono allineati con quelli del frame dell'end-effector.
  // Per il controllo nello spazio operativo, spesso si usa WORLD o LOCAL_WORLD_ALIGNED.
  // Scegli quello consistente con come definisci l'errore e le velocità/accelerazioni desiderate.
  // Assumiamo WORLD per ora, implicando che X_d_dot e X_d_ddot siano nel frame base di Pinocchio.
  pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_pin_, pinocchio::WORLD, J_base);

  Eigen::MatrixXd dJ_base(6, model_.nv);
  dJ_base.setZero();
  pinocchio::getFrameJacobianTimeVariation(model_, *data_, ee_frame_id_pin_, pinocchio::WORLD, dJ_base);

  // LOGGING: Valori di Pinocchio (Jacobiano, M, N) -- AGGIUNTO
  RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), THROTTLE_MS, "Jacobian J(0,0): %.3f | M(0,0): %.3f | nle(0): %.3f", J_base(0,0), data_->M(0,0), data_->nle(0));

  // --- E. LEGGE DI CONTROLLO ---

  // 1. Calcola Errore nello Spazio Operativo X_e (nel frame base di Pinocchio)
  //    X_e rappresenta la trasformazione per andare da X_actual_pin_base a X_d_pin_base
  //pinocchio::SE3 X_e_pin_base = X_actual_pin_base.actInv(X_d_pin_base); // Errore: X_attuale_inv * X_desiderata
  //Eigen::Matrix<double, 6, 1> error_se3_vec = pinocchio::log6(X_e_pin_base).toVector(); // Vettore errore 6D (porta da attuale a desiderato)
  // MODIFICA PER DETERMINARE L'ERRORE PRIMA IN LOCALE E POI IN WORLD -- AGGIUNTA7  -- COMMENTATE LE RIGHE DI SOPRA
  pinocchio::SE3 X_e_pin_base = X_actual_pin_base.actInv(X_d_pin_base);
  pinocchio::Motion error_pose_local = pinocchio::log6(X_e_pin_base);
  Eigen::Matrix<double, 6, 1> error_pose_local_vec = error_pose_local.toVector();
  Eigen::Matrix<double, 6, 1> error_pose_world_vec = X_actual_pin_base.toActionMatrix() * error_pose_local_vec;

  // LOGGING: Errore -- AGGIUNTO
  //RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), THROTTLE_MS, "Vettore Errore 6D: Lin=[%.3f, %.3f, %.3f], Ang=[%.3f, %.3f, %.3f]", error_se3_vec(0), error_se3_vec(1), error_se3_vec(2), error_se3_vec(3), error_se3_vec(4), error_se3_vec(5));
  // DA COMMENTARE SE CAMBI COME SCRIVI L'ERRORE SOPRA (AGGIUNTA7)

  // --- LOGGING DI DEBUG PER PINOCCHIO (MATRICI COMPLETE) --- AGGIUNTO2
  // Converti le matrici e i vettori Eigen in stringhe per il logging
  std::stringstream ss_log;
  ss_log << "\n--- Pinocchio Debug (T=" << std::fixed << std::setprecision(3) << time.seconds() << ") ---\n"
        << "Jacobian J_base (6x" << model_.nv << "):\n" << J_base << "\n"
        << "----------------------------------------------------------\n"
        << "Mass Matrix M (" << model_.nv << "x" << model_.nv << "):\n" << data_->M << "\n"
        << "----------------------------------------------------------\n"
        << "Non-Linear Effects nle (C(q,dq)*dq + G(q)):\n" << data_->nle.transpose() << "\n"
        << "----------------------------------------------------------";
  // Stampo ogni 4000 ms
  RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 4000, "%s", ss_log.str().c_str());
  // --- FINE LOGGING DI DEBUG ---

  // 2. Calcola Velocità Attuale EE (nel frame base di Pinocchio, coerente con J_base)
  //pinocchio::Motion X_actual_dot_pin_base = pinocchio::getFrameVelocity(model_, *data_, ee_frame_id_pin_, pinocchio::WORLD);

  // 3. Calcola Errore di Velocità (nel frame base di Pinocchio)
  //    Trasforma Xd_dot_map nel frame base di Pinocchio se necessario. Assumendo siano già coerenti per ora.
  //    pinocchio::Motion Xd_dot_pin_base = X_base_map.act(Xd_dot_map); // Se Xd_dot_map è in 'map'
  //pinocchio::Motion Xd_dot_pin_base = Xd_dot_map; // Semplificazione, DA VERIFICARE FRAME!
  //Eigen::Matrix<double, 6, 1> error_vel_vec = Xd_dot_pin_base.toVector() - X_actual_dot_pin_base.toVector();

  // MODIFICA PER DETERMINARE L'ERRORE PRIMA IN LOCALE E POI IN WORLD -- AGGIUNTA7  -- COMMENTATE LE TRE RIGHE DI SOPRA
  pinocchio::Motion vel_actual_world = pinocchio::getFrameVelocity(model_, *data_, ee_frame_id_pin_, pinocchio::WORLD);
  pinocchio::Motion vel_error_world = Xd_dot_map - vel_actual_world; // Assumendo Xd_dot_map sia nel frame mondo

  // LOGGING: Errore -- AGGIUNTO3
  //RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), THROTTLE_MS, "Errore Vel 6D: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", error_vel_vec(0), error_vel_vec(1), error_vel_vec(2),error_vel_vec(3), error_vel_vec(4), error_vel_vec(5));
  // DA COMMENTARE SE CAMBI COME SCRIVI L'ERRORE SOPRA (AGGIUNTA7)

  // 4. Calcola l'Accelerazione Cartesiana Desiderata $\ddot{X}_{cmd}$
  //    Questa è la PARTE CENTRALE da implementare basandosi sul paper (funzione potenziale U, gradiente phi, ecc.)
  //    Per ora, un PD+feedforward accelerazione (semplificato):
  Eigen::Matrix<double, 6, 1> ddX_cmd_base;
  Eigen::Matrix<double, 6, 6> Kp_mat = Eigen::Matrix<double, 6, 6>::Zero();
  Kp_mat.diagonal() << kp_linear_, kp_linear_, kp_linear_, kp_angular_, kp_angular_, kp_angular_;
  Eigen::Matrix<double, 6, 6> Kd_mat = Eigen::Matrix<double, 6, 6>::Zero();
  Kd_mat.diagonal() << kd_linear_, kd_linear_, kd_linear_, kd_angular_, kd_angular_, kd_angular_;

  // Accelerazione desiderata = Acc_FF + Kp*(Pos_des - Pos_act) + Kd*(Vel_des - Vel_act)
  // Dato che error_se3_vec porta da X_actual a X_desired, usiamo Kp * error_se3_vec
  // Dato che error_vel_vec è Xd_dot - X_actual_dot, usiamo Kd * error_vel_vec
  // pinocchio::Motion Xd_ddot_pin_base = X_base_map.act(Xd_ddot_map); // Se Xd_ddot_map è in 'map'
  //pinocchio::Motion Xd_ddot_pin_base = Xd_ddot_map; // Semplificazione, DA VERIFICARE FRAME!
  //ddX_cmd_base = Xd_ddot_pin_base.toVector() + Kp_mat * error_se3_vec + Kd_mat * error_vel_vec;
  // MODIFICA PER USARE SOLO GRANDEZZE IN WORLD -- AGGIUNTA7  -- COMMENTATE LE DUE RIGHE DI SOPRA
  ddX_cmd_base = Xd_ddot_map.toVector() + Kp_mat * error_pose_world_vec + Kd_mat * vel_error_world.toVector();

  // --- F. CALCOLO COPPIE (Computed Torque Control) ---

  // 1. Calcola Accelerazioni dei Joint Desiderate ($\ddot{q}_{des}$)
  //    $\ddot{q}_{des} = J^{\dagger}(\ddot{X}_{cmd} - \dot{J}\dot{q})$
  Eigen::MatrixXd J_base_transpose = J_base.transpose();
  double damping_pinv = 0.01; // Valore di smorzamento per la pseudoinversa, da tunare
  Eigen::MatrixXd J_base_pseudo_inv = J_base_transpose * (J_base * J_base_transpose + damping_pinv * Eigen::Matrix<double,6,6>::Identity()).inverse();

  Eigen::VectorXd ddq_desired = J_base_pseudo_inv * (ddX_cmd_base - dJ_base * dq_);

  // 2. Calcola Coppie Comandate $\tau^*$
  //    $\tau^* = M(q) \ddot{q}_{des} + N(q,\dot{q})$
  //    data_->M (matrice di inerzia) e data_->nle (Coriolis+gravità) sono già stati calcolati da computeAllTerms()
  tau_ = data_->M * ddq_desired + data_->nle;

  // LOGGING: Coppie Finali  -- AGGIUNTO
  RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 250, "Coppie Finali tau*: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", tau_(0), tau_(1), tau_(2), tau_(3), tau_(4), tau_(5));

  // Usa uno stringstream per costruire il log con le matrici complete -- AGGIUNTO3
  std::stringstream ss_log_matrices;
  ss_log_matrices << std::fixed << std::setprecision(4); // Imposta precisione decimale per la leggibilità
  ss_log_matrices << "\n================ UPDATE CYCLE (T=" << time.seconds() << ") ================\n"
                  << "--- JACOBIANE ---\n"
                  << "Jacobian Pseudoinversa J_pinv (6x" << model_.nv << "):\n" << J_base_pseudo_inv << "\n\n" // <-- STAMPA DELLA PSEUDOINVERSA
                  << "--- DINAMICA E COMANDI ---\n"
                  << "Accelerazione Joint Desiderata ddq_des: " << ddq_desired.transpose() << "\n"
                  << "=========================================================";
  // Stampa il log completo delle matrici con una frequenza più bassa
  RCLCPP_INFO_STREAM_THROTTLE(logger, *get_node()->get_clock(), 4000,ss_log_matrices.str());
  // --- FINE LOGGING ---


  // --- LOGICA DI AVANZAMENTO WAYPOINT (BASATA SU TOLLERANZA) --- AGGIUNTO5
  // Controlla se il waypoint attuale è stato raggiunto
  //double position_error_norm = error_se3_vec.head<3>().norm();
  double position_error_norm = error_pose_world_vec.head<3>().norm(); // <-- AGGIUNTA7 COERENTE CON NOME DEL NUOVO ERRORE
  double goal_tolerance = 0.001; // Tolleranza di 1 cm (puoi metterla come parametro ROS)

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                         "Inseguendo waypoint #%zu. Errore di posizione: %.4f m",
                         current_trajectory_index_, position_error_norm);

  //Aggiungo un if per non far avanzare all'infinito l'indice della traiettoria e poter mantenere l'ultimo waypoint ciclico -- AGGIUNTO9
  //Sposto di due spazi il blocco interno di codice
  if (trajectory_active_) {
  // Se sei abbastanza vicino al target attuale, avanza al prossimo
    if (position_error_norm < goal_tolerance) {
        current_trajectory_index_++;
        if (current_trajectory_index_ < desired_trajectory_.size()) {
            RCLCPP_INFO(get_node()->get_logger(), "Waypoint raggiunto! Avanzamento a waypoint #%zu.", current_trajectory_index_);
        } else {
            RCLCPP_INFO(get_node()->get_logger(), "TRAIETTORIA COMPLETATA!");
            trajectory_active_ = false; // Ferma l'inseguimento attivo
        }
    }
  }
  //FINE AGGIUNTO5

  // --- G. INVIO COMANDI E AGGIORNAMENTI ---
  // (Opzionale) Controllo di Collisione Finale qui, potrebbe modificare tau_

  // Scrivi Comandi di Sforzo ai Joint
  if (tau_.size() == static_cast<long int>(num_joints_) && cmd_if_effort_handles_.size() == num_joints_) {
      for (size_t i = 0; i < num_joints_; ++i) {
          cmd_if_effort_handles_[i].get().set_value(tau_(i));
      }
  } else {
      RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000, 
          "Dimensioni errate per comandi di coppia (%ld) o interfacce di comando (%zu). Giunti attesi: %zu",
          tau_.size(), cmd_if_effort_handles_.size(), num_joints_);
      // Invia comandi sicuri
      for (size_t i = 0; i < cmd_if_effort_handles_.size(); ++i) { command_interfaces_[i].set_value(0.0); }
      return controller_interface::return_type::ERROR; // Errore se le dimensioni non corrispondono
  }

  return controller_interface::return_type::OK;
}

}  // namespace op_space_controller

// plugin export
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  op_space_controller::OpSpaceController,
  controller_interface::ControllerInterface)

