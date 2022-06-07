

//var t =setInterval(actualizar, 100); // 1000 = 1 segundo


document.addEventListener('DOMContentLoaded', event => {
    window.onload = connect
    document.getElementById("btn_forward").addEventListener("click", () => {call_service("delante")})
    document.getElementById("btn_backward").addEventListener("click", () => {call_service("atras")})
    document.getElementById("btn_left").addEventListener("click", () => {call_service("izquierda")})
    document.getElementById("btn_right").addEventListener("click", () => {call_service("derecha")})
    document.getElementById("btn_stop").addEventListener("click", () => {call_service("parar")})
    document.getElementById('btn-video').addEventListener("click", actualizar)
    let stream_state
  
    let data = {
      // ros connection
      ros: null,
      rosbridge_address: 'ws://127.0.0.1:9090/',
      connected: false,
      // service information 
      service_busy: false,
      service_response: ''
    }
  
    let direction = 1
    let stream_count = 0
  
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

    function actualizar() {
        
        stream_count++;
        if(stream_count%2 != 0){
            console.log("Streaming activo")
            stream_state = setInterval(function(){
                
                var tiempo = new Date();
                var rnd =tiempo.getTime();
                var imagen = document.getElementById('my_image');
                imagen.src = '../imgs/power.png?'+rnd;
            }, 100)
            
        } else{
            console.log("Streaming inactivo")
            clearInterval(stream_state)
            imagen.src = '../imgs/camera.png';
        }
        
    }
        
    
  
    function call_service(valor){
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
  
   
  });
  