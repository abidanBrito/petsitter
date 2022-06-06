const boton = document.querySelector('#btn-video')

function toggleButton() {
    if (boton.classList.contains('encender')) {
        boton.className = "btn btn-primary apagar"
        boton.textContent = "Apagar"
        //console.log("clase: " + boton.className)
    }  
    else if (boton.classList.contains('apagar')) {
        boton.textContent = "Encender"
        boton.className = "btn btn-primary encender"
        //console.log("clase: " + boton.className)
    }
}