document.addEventListener('DOMContentLoaded', event => {
  if (document.getElementById("btn_dis").classList.contains('power')) { // si el boton rojo contiene la clase power
    document.getElementById("btn_dis").removeEventListener("click", disconnect)
    document.getElementById("btn_dis").addEventListener("click", connect)
    //document.getElementById("btn_dis").addEventListener("click", subscribe)

  } else if (document.getElementById("btn_dis").classList.contains('power2')) { // si el boton verde contiene la clase power2
    document.getElementById("btn_dis").removeEventListener("click", connect)
    document.getElementById("btn_dis").addEventListener("click", disconnect)
  }

  /*document.getElementById("btn_forward").addEventListener("click", () => {call_service("delante")})
  document.getElementById("btn_backward").addEventListener("click", () => {call_service("atras")})
  document.getElementById("btn_left").addEventListener("click", () => {call_service("izquierda")})
  document.getElementById("btn_right").addEventListener("click", () => {call_service("derecha")})
  document.getElementById("btn_stop").addEventListener("click", () => {call_service("parar")})*/
  document.getElementById("comedero").addEventListener("click", send_pose_service_cuenco)
  document.getElementById("carga").addEventListener("click", send_pose_service_carga)
  document.getElementById("btn-feeder").addEventListener("click", detect_food)
  document.getElementById("escaneado").addEventListener("click", scan_map_toggle)

  let data = {
    // ros connection
    ros: null,
    rosbridge_address: 'ws://127.0.0.1:9090/',
    connected: false,
    // service information 
    service_busy: false,
    service_response: '',
    scanning: false
  }

  let direction = 1

  function connect() {
    data.ros = new ROSLIB.Ros({
      url: data.rosbridge_address
    })

    // Define callbacks
    data.ros.on("connection", () => {
      data.connected = true
      console.log("Conexion con ROSBridge correcta")
    })
    data.ros.on("error", (error) => {
      console.log("Se ha producido algun error mientras se intentaba realizar la conexion")
      console.log(error)
    })
    data.ros.on("close", () => {
      data.connected = false
      console.log("Conexion con ROSBridge cerrada")
    })
  }

  function disconnect() {
    data.ros.close()
    data.connected = false
    console.log('Clic en botón de desconexión')
  }

  function move() {
    let topic = new ROSLIB.Topic({
      ros: data.ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist'
    })
    let message = new ROSLIB.Message({
      linear: { x: 0.1, y: 0, z: 0, },
      angular: { x: 0, y: 0, z: -0.2, },
    })
    topic.publish(message)
  }

  function turn() {
    let topic = new ROSLIB.Topic({
      ros: data.ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist'
    })
    let message = new ROSLIB.Message({
      linear: { x: 0.1, y: 0, z: 0, },
      angular: { x: 0, y: 0, z: -0.2 * direction, },
    })
    topic.publish(message)

    direction = -1 * direction
  }

  function stop() {
    let topic = new ROSLIB.Topic({
      ros: data.ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist'
    })
    let message = new ROSLIB.Message({
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    })
    topic.publish(message)
  }

  function detect_food() {
    data.service_busy = true
    data.service_response = ''

    //definimos los datos del servicio
    let service = new ROSLIB.Service({
      ros: data.ros,
      name: '/analize',
      serviceType: 'petsitter_custom_interface/srv/OpencvMsg'
    })

    let request = new ROSLIB.ServiceRequest({
      analize: true

    })

    service.callService(request, (result) => {
      data.service_busy = false
      data.service_response = JSON.stringify(result)
    }, (error) => {
      data.service_busy = false
      console.error(error)
    })
  }

  /*function set_initial_pos() {
    let topic = new ROSLIB.Topic({
      ros: data.ros,
      name: '/initialpose',
      messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped'
    })
    let message = new ROSLIB.Message({
      header: 'map',
      pose: {x: 0.1, y: 0, z: 0},
    })
    topic.publish(message)
  }*/

  /*function subscribe() {
    let topic = new ROSLIB.Topic({
      ros: data.ros,
      name: '/odom',
      messageType: 'nav_msgs/msg/Odometry'
    })
    topic.subscribe((message) => {
      data.position = message.pose.pose.position
      document.getElementById("pos_x").innerHTML = data.position.x.toFixed(2)
      document.getElementById("pos_y").innerHTML = data.position.y.toFixed(2)
    })
  }*/

  function call_service(valor) {
    data.service_busy = true
    data.service_response = ''

    //definimos los datos del servicio
    let service = new ROSLIB.Service({
      ros: data.ros,
      name: '/movement',
      serviceType: 'custom_interface/srv/MyMoveMsg'
    })

    let request = new ROSLIB.ServiceRequest({
      move: valor
    })

    service.callService(request, (result) => {
      data.service_busy = false
      data.service_response = JSON.stringify(result)
    }, (error) => {
      data.service_busy = false
      console.error(error)
    })
  }

  function send_pose_service_carga() {
    data.service_busy = true
    data.service_response = ''

    //definimos los datos del servicio
    let service = new ROSLIB.Service({
      ros: data.ros,
      name: '/send_pose',
      serviceType: 'custom_interface/srv/NavMessage'
    })

    let request = new ROSLIB.ServiceRequest({
      x: 6.12,
      y: -0.2

    })

    service.callService(request, (result) => {
      data.service_busy = false
      data.service_response = JSON.stringify(result)
    }, (error) => {
      data.service_busy = false
      console.error(error)
    })
  }

  function send_pose_service_cuenco() {
    data.service_busy = true
    data.service_response = ''

    //definimos los datos del servicio
    let service = new ROSLIB.Service({
      ros: data.ros,
      name: '/send_pose',
      serviceType: 'custom_interface/srv/NavMessage'
    })

    let request = new ROSLIB.ServiceRequest({
      x: 6,
      y: -6.1

    })

    service.callService(request, (result) => {
      data.service_busy = false
      data.service_response = JSON.stringify(result)
    }, (error) => {
      data.service_busy = false
      console.error(error)
    })
  }

  // NOTE(abi): this function is meant to toggle SLAM on and off.
  function scan_map_toggle() {
    if (data.connected == false) {
      connect();
    }

    try {
      // Limpiamos la respuesta (en caso de que hubiese alguna)
      // y ponemos el servicio en uso
      data.service_busy = true
      data.service_response = ''

      // Servicio del SLAM autónomo
      let slamService = new ROSLIB.Service({
        ros: data.ros,
        name: '/autonomous_slam',
        serviceType: 'petsitter_custom_interface/srv/ScanMsg'
      })

      // Petición  
      let msg = (data.scanning == false) ? "start" : "stop"
      let request = new ROSLIB.ServiceRequest({
        scan: msg
      })

      // Realizamos la llamada al servicio
      slamService.callService(request, (res) => {
        data.service_busy = false
        data.scanning = (data.scanning == true) ? false : true
        console.log("Scanning " + data.scanning)
        data.service_response = JSON.stringify(res)
      }, (err) => {
        data.service_busy = false
        console.error("Service call failed - " + err)
      })
    }
    catch (err) {
      console.error("Start scan order failed! - " + err)
    }
  }

});