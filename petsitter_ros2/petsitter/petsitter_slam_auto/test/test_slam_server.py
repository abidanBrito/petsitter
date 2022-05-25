#################################################################################
# Autor: Abidán Brito
# Fecha: 25/05/2022
# Nombre del fichero: test_slam_server.py
# Descripción: Tests unitarios de la clase SlamService.
#################################################################################

# Import file from relative path
import sys
from urllib import request, response
sys.path.insert(1, '../petsitter_slam_auto')
from slam_server import SlamService as ss

def test_found_collision():
    """
    Test del método privado found_collision de la clase SlamService, que detecta
    colisiones antes de que sucedan, con un umbral de distancia que se le pasa como
    parámetro, junto a la lista de distancias para los distintos grados.
    """
    assert ss._found_collision(ss, [1.0, 2.0, 4.0], 0.5) == False
    assert ss._found_collision(ss, [0.7, 0.4, 2.1], 0.5) == True
    assert ss._found_collision(ss, [0.61, 0.8, 1.3], 0.6) == False