// Autor: Abidán Brito Clavijo
// Fecha: 07/06/2022

import { initializeApp } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-app.js";
import { getStorage, ref, getDownloadURL } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-storage.js";
import { getFirestore, doc, getDoc } from "https://www.gstatic.com/firebasejs/9.8.1/firebase-firestore.js";

const firebaseConfig = {
    apiKey: "AIzaSyABOCK6NTYF1jsv1WT5SiDZdFgUuhXLH2o",
    authDomain: "petsitter-42488.firebaseapp.com",
    projectId: "petsitter-42488",
    storageBucket: "petsitter-42488.appspot.com",
    messagingSenderId: "590637458117",
    appId: "1:590637458117:web:e0681e78d84b708a3c46ac"
};

const app = initializeApp(firebaseConfig);
const storage = getStorage()
const db = getFirestore(app);

document.addEventListener('DOMContentLoaded', event => {
    document.getElementById("btn-map").addEventListener("click", loadMap);
    let canvas = document.getElementById("canvas");
    let ctx = canvas.getContext("2d");
    let mapStr = "";

    async function loadMap() {
        try {
            const docRef = doc(db, "maps", "robot-500");
            const docSnap = await getDoc(docRef);

            if (docSnap.exists()) {
                document.getElementById("map-hint").style.display = 'none'; 
                let img = new Image();

                img.onload = function () {
                    ctx.drawImage(this, 0, 0, canvas.width, canvas.height);
                }
                
                img.src = "data:image/gif;base64," + docSnap.data()["base64"];
            } 
            else {
                document.getElementById("map-hint").style.display = 'block'; 
                console.error("No existe el mapa.");
            }

            canvas.addEventListener("click", function (event) {
                ctx.beginPath();
                let pos = getMousePosition(event, canvas);
                console.log("You clicked on: " + JSON.stringify(pos))
            }, false)
        }
        catch (err) {
            console.log(err);
        }
    }

    function getMousePosition(event, canvas) {
        var rectangle = canvas.getBoundingClientRect();
        return {
            x: Math.floor(event.clientX - rectangle.left),
            y: Math.floor(event.clientY - rectangle.top)
        };
    }

    /*
    function canvasPointToRosPoint(xCoord, yCoord, canvasMaxX, canvasMaxY, canvas, resolution) {
        let maxY = canvasMaxY * resolution;
        return {
            x: xCoord * resolution * canvasMaxX / canvas.width,
            y: maxY - yCoord * resolution * canvasMaxY / canvas.height
        }
    }
    */
});

