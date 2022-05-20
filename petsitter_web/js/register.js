import { initializeApp } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-app.js";
import { getAuth, createUserWithEmailAndPassword } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-auth.js";
import { getFirestore, collection, query, getDocs } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-firestore.js";

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

const registerForm = document.getElementById('signup-form')

// Función de registro

registerForm.addEventListener('submit', async (e) => {
    
    e.preventDefault(); // evitar que recargue la página

    const email = document.getElementById('email-r').value
    const password = document.getElementById('password-r').value
    const code = document.getElementById('code-r').value
    const messageBox = document.getElementById('message_box')
    const q = query(collection(db, "Codigos"))
    const querySnapshot = await getDocs(q)
    let codigos = []

    // Añadir los codigos a una lista
    querySnapshot.forEach((doc) => {
        codigos.push(doc.data()['codigo'])
    })

    console.log(codigos)

    if (codigos.includes(code)) { // Si el código que introduce el usuario está en la lista:

        // Crear usuario
        createUserWithEmailAndPassword(auth, email, password)
        .then( (userCredential) => {
            console.log("Usuario registrado.")
            document.location.href = 'login.html'
        })
        .catch((error) => {
            const errorCode = error.code;
            const errorMessage = error.message;
            if (password.length < 6) {
                messageBox.innerHTML = "La contraseña debe tener al menos 6 caracteres."
                console.error('Algo falló: ' + errorMessage)
            }
        })

    }
    else {
        messageBox.innerHTML = "El código no es correcto."
        console.error('Código incorrecto.')
    }

    

})
