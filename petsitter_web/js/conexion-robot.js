const power = document.getElementById('btn_dis')
const texto = document.getElementById('conn-text')

function toggleButton() {
    if (texto.innerHTML == "Conectado") {
        texto.innerHTML = "Desconectado";
        power.id = "btn_dis"
        console.log("La id es: " + power.id)
      } else {
        texto.innerHTML = "Conectado";
        power.id = "btn_con"
        console.log("La id es: " + power.id)
      }
}
    
