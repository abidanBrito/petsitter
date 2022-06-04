/*import { initializeApp } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-app.js";
import { getStorage, ref, getDownloadURL } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-storage.js";
import { getFirestore, doc, getDoc } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-firestore.js";

// Configuración Firebase

const firebaseConfig = {

    apiKey: "AIzaSyABOCK6NTYF1jsv1WT5SiDZdFgUuhXLH2o",

    authDomain: "petsitter-42488.firebaseapp.com",

    projectId: "petsitter-42488",

    storageBucket: "petsitter-42488.appspot.com",

    messagingSenderId: "590637458117",

    appId: "1:590637458117:web:e0681e78d84b708a3c46ac"

};

// Inicializar Firebase

const app = initializeApp(firebaseConfig);
const storage = getStorage()
const db = getFirestore(app);

// -----------------------------------------------

const btnComedero = document.getElementById('btn-feeder') // boton para tomar fotos del comedero

btnComedero.addEventListener('click', (e) => {

    // Obtener imagen de storage desde 'images/[nombre_archivo_imagen]'

    getDownloadURL(ref(storage, 'images/Imagen_cuenco.jpg')) 
    .then(async (url) => {

        // Insertar imagen con url en elemento <img>
        const img = document.getElementById('img-feeder');
        img.setAttribute('src', url);
        

    })
    .catch((error) => {
      // Handle any errors
      console.error("Algo falló: " + error)
    })

})
*/
document.addEventListener('DOMContentLoaded', event => {
  if(document.getElementById("btn_dis")){
    document.getElementById("btn_dis").addEventListener("click", connect)
    
  } else if(document.getElementById("btn_con")){
    document.getElementById("btn_dis").removeEventListener("click", connect)
    document.getElementById("btn_con").addEventListener("click", disconnect)
  }

  document.getElementById("btn_forward").addEventListener("click", () => {call_service("delante")})
  document.getElementById("btn_backward").addEventListener("click", () => {call_service("atras")})
  document.getElementById("btn_left").addEventListener("click", () => {call_service("izquierda")})
  document.getElementById("btn_right").addEventListener("click", () => {call_service("derecha")})
  document.getElementById("btn_stop").addEventListener("click", () => {call_service("parar")})
  document.getElementById("comedero").addEventListener("click", send_pose_service_cuenco)
  document.getElementById("carga").addEventListener("click", send_pose_service_carga)
  document.getElementById("btn-detect").addEventListener("click", detect_pets)

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

  function send_pose_service_carga(){
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

  function send_pose_service_cuenco(){
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

  function detect_pets(){
    data.service_busy = true
    data.service_response = ''	
  
    //definimos los datos del servicio
    let service = new ROSLIB.Service({
        ros: data.ros,
        name: '/detect',
        serviceType: 'petsitter_custom_interface/srv/DetectMsg'
    })
  
    let request = new ROSLIB.ServiceRequest({
        detect: true
      
    })
  
    service.callService(request, (result) => {
        console.log("Detection mode: ON")
        data.service_busy = false
        data.service_response = JSON.stringify(result)
    }, (error) => {
        data.service_busy = false
        console.error(error)
    }) 
  }
});
