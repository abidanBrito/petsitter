const dis = document.getElementById('btn_dis')
const texto = document.getElementById('conn-text')

function toggleButton() {
    if (texto.innerHTML == "Conectado") {
        texto.innerHTML = "Desconectado";
        dis.className = "power"
      } else {
        texto.innerHTML = "Conectado";
        dis.className = "power2"
      }
}
