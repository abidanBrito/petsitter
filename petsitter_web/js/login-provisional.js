    var usuarios = ["usuario"]
    var contraseñas = ["1234"]
    var codigos = ["codigo"]
    //Se obtiene los botones en js con getElementById
    btn_login = document.getElementById("login-btn")
    btn_register = document.getElementById("register-btn")
    btn_login.onclick = login
    // btn_register.onclick = register

    function login() {
        var color = "#ced4da"
        document.getElementById("username").style.borderColor = color
        document.getElementById("password").style.borderColor = color

        var user = document.getElementById("username").value
        var pass = document.getElementById("password").value
        console.log(user)
        var index = usuarios.indexOf(user);
        if (index != -1) {
            var indexP = contraseñas.indexOf(pass);
               if (indexP != -1 && indexP == index) {
                    window.location.href = "control.html";
                } else {
                    color = "#FF0000"
                    document.getElementById("password").style.borderColor = color
                }
        } else {
            color = "#FF0000"
            document.getElementById("username").style.borderColor = color
        }
    }
