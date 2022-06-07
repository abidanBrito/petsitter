import { initializeApp } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-app.js";
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
const btnMascota = document.getElementById('btn-petPhoto') // boton para tomar fotos de la mascota
const prog = document.getElementById('progress-bar') // barra de cantidad de comida
const fecha = document.getElementById('fecha') // texto fecha de la ultima foto
const textoDeteccion = document.getElementById('detect-text') // texto para el tipo de mascota
const textoMascota = document.getElementById('pet-text') // tipo mascota detectada
let amount = null; // cantidad de comida que obtenemos de Firestore
let lastDate = ''; // fecha de la última foto
let pet = ''; // tipo de mascota

window.onload = descargarDatos()
//window.onload = descargarMascota()

btnComedero.addEventListener('click', (e) => { // al clicar el botón de Tomar foto:

  // Se descargan los últimos datos subidos
  descargarDatos()

})

btnMascota.addEventListener('click', (e) => { // al clicar el botón de Tomar foto:

  // Se descargan los últimos datos subidos
  textoDeteccion.removeAttribute("hidden");
  descargarMascota()

})

function descargarDatos() {

  // Subir imagen a storage en 'images/[nombre_archivo_imagen]'

  getDownloadURL(ref(storage, 'images/Imagen_cuenco.jpg')) // path de la imagen en Storage
  .then(async (url) => {

         // Obtener cantidad de comedero desde Firestore
         const docRef = doc(db, "Imagenes", "cuenco");
         const docSnap = await getDoc(docRef);
 
         if (docSnap.exists()) {
           amount = docSnap.data()["cantidad"]
           lastDate = docSnap.data()["fecha"]
           //console.log("cantidad = " + amount) 
         } else {
           // doc.data() will be undefined in this case
           console.error("No existe el documento 'imagen'");
         }
 
         // Cambiar cantidad de comedero
         prog.style.width = amount + "%";

         // Cambiar fecha
        fecha.innerHTML = lastDate;
 
         // Insertar imagen con url en elemento <img>
         const img = document.getElementById('img-feeder');
         img.setAttribute('src', url);
         setTimeout( () => { 
           img.style.width = "300px";
           img.style.height = "150px";
           img.style.borderRadius = "10px";
         }, 100);
    })
    .catch((error) => {
      // Handle any errors
      console.error("Algo falló: " + error)
    })
}

function descargarMascota() {

  // Subir imagen a storage en 'images/[nombre_archivo_imagen]'

  getDownloadURL(ref(storage, 'images/mascota.jpg')) // path de la imagen en Storage
  .then(async (url) => {

         // Obtener mascota desde Firestore
         const docRef = doc(db, "Imagenes", "mascota");
         const docSnap = await getDoc(docRef);
 
         if (docSnap.exists()) {
           pet = docSnap.data()["tipo de mascota"]
           //console.log("cantidad = " + amount) 
         } else {
           // doc.data() will be undefined in this case
           console.error("No existe el documento 'imagen'");
         }

         // Cambiar tipo de mascota
        textoMascota.innerHTML = pet;
 
         // Insertar imagen con url en elemento <img>
         const img = document.getElementById('img-pet');
         img.setAttribute('src', url);
         setTimeout( () => { 
           img.style.width = "300px";
           img.style.height = "150px";
           img.style.borderRadius = "10px";
         }, 100);
    })
    .catch((error) => {
      // Handle any errors
      console.error("Algo falló: " + error)
    })
}
