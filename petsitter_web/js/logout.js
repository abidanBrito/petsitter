import { initializeApp } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-app.js";
import { getAuth, signOut } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-auth.js";

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

// ---------------------------------------------------------------------

const logoutButton = document.getElementById('logout')

// Función de cerrar sesión

logoutButton.addEventListener('click', (e) => {
    
    e.preventDefault(); // evitar que recargue la página

    signOut(auth)
    .then(() => {
        console.log('Se ha cerrado la sesión.')
        window.location.href = 'index.html'
    }).catch((error) => {
        const errorCode = error.code;
        const errorMessage = error.message;
        console.error('Algo falló: ' + errorMessage)
    });
      
})