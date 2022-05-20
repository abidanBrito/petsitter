import { initializeApp } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-app.js";
import { getAuth, signInWithEmailAndPassword } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-auth.js";
import { getFirestore, collection, addDoc } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-firestore.js";

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
const auth = getAuth(app);
const db = getFirestore(app);

// ---------------------------------------------------------------------

const loginForm = document.getElementById('login-form')

// Función de registro

loginForm.addEventListener('submit', (e) => {
    
    e.preventDefault(); // evitar que recargue la página

    const email = document.getElementById('email').value;
    const password = document.getElementById('password').value;
    const messageBox = document.getElementById('message_box')

    // Comprobar usuario
    signInWithEmailAndPassword(auth,email,password)
    .then((userCredentials) => {
        const user = userCredentials.user;
        console.log(email + ' ha iniciado sesión correctamente.')
        window.location.href = 'control.html'
    })
    .catch((error) => {
        const errorCode = error.code;
        const errorMessage = error.message;
        messageBox.innerHTML = "Correo o contraseña incorrectos."
        console.error('Algo falló: ' + errorMessage)
    })

})
