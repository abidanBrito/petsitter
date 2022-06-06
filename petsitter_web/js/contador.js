const btnIniciar = document.getElementById("escaneado")
const contador = document.getElementById("timer-count")


let idInterval = null,
    diferenciaTemporal = 0, 
    fechaFuturo = null;

const iniciarTemporizador = (minutos,segundos) => {
    if (fechaFuturo) {
        fechaFuturo = new Date(new Date().getTime() + diferenciaTemporal);
        diferenciaTemporal = 0;
    } else {
        const milisegundos = (segundos + (minutos*60) * 1000);
        fechaFuturo = new Date(new Date().getTime() + milisegundos);
    }
    clearInterval(idInterval);
    idInterval = setInterval(() => {
        const tiempoRestante = fechaFuturo.getTime() - new Date().getTime();
        if (tiempoRestante <= 0) {
            clearInterval(idInterval);
        } else {
            contador.textContent = milisegundosAMinutosYSegundos(tiempoRestante);
        }
    }, 50);
}

const milisegundosAMinutosYSegundos = (milisegundos) => {
    const minutos = parseInt(milisegundos / 1000 / 60);
    milisegundos -= minutos * 60 * 1000;
    segundos = (milisegundos / 1000);
    return `${agregarCeroSiEsNecesario(minutos)}:${agregarCeroSiEsNecesario(segundos.toFixed(1))}`;
}

const agregarCeroSiEsNecesario = (valor) => {
    if (valor < 10) {
        return "0" + valor;
    } else {
        return "" + valor;
    }
}

btnIniciar.addEventListener("click", () => {
    iniciarTemporizador(5,0);
})