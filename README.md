# 🚁 ROS Workspace - Control del Drone Parrot Bebop 2

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Python 3](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)

Este repositorio contiene un **workspace ROS Noetic** con paquetes para controlar un dron **Parrot Bebop 2** mediante ROS.
Incluye instalación desde cero, configuración del entorno, comandos básicos de vuelo y ejemplos de Python.


---

<a id="indice"></a>

## 📋 Índice
- [📦 Estructura del Proyecto](#estructura-del-proyecto)
- [✅ Requisitos](#requisitos)
- [🔧 Instalación desde Cero](#instalacion-desde-cero)
- [▶️ Uso del Drone Parrot Bebop 2](#uso-del-drone-parrot-bebop-2)
  - [1️⃣ Conexión con el dron](#conexion-con-el-dron)
  - [2️⃣ Iniciar ROS](#iniciar-ros)
  - [3️⃣ Lanzar el Nodo Principal](#lanzar-el-nodo-principal)
  - [4️⃣ Comandos Básicos](#comandos-basicos)
  - [5️⃣ Verificar Tópicos Disponibles](#verificar-topicos-disponibles)
  - [6️⃣ Ver la Cámara](#ver-la-camara)
  - [7️⃣ Visualizar Nodos y Tópicos (rqt_graph)](#visualizar-nodos-y-topicos-rqt-graph)
  - [8️⃣ Ejemplo Python - Vuelo Simple](#ejemplo-python-vuelo-simple)
  - [9️⃣ Diagrama Básico del Flujo de Vuelo](#diagrama-basico-del-flujo-de-vuelo)



 
---

<a id="estructura-del-proyecto"></a>

## 📦 Estructura del Proyecto

```
bebop_ws/
 ├── build/
 ├── devel/
 └── src/
      ├── parrot_arsdk       # Wrapper SDK Parrot
      └── bebop_autonomy     # Driver principal ROS
```

[🔙 Volver al Índice](#indice)

---

<a id="requisitos"></a>

## ✅ Requisitos

* **Sistema operativo:** Ubuntu 20.04 LTS
* **ROS Noetic** → [Instalación oficial](http://wiki.ros.org/noetic/Installation/Ubuntu)

**Dependencias ROS:**

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-cmake-modules \
    ros-noetic-image-transport \
    ros-noetic-diagnostic-updater \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-message-filters \
    ros-noetic-camera-info-manager
```

**Librerías del sistema:**

```bash
sudo apt-get install -y \
    build-essential cmake git python3-catkin-tools \
    libavcodec-dev libavdevice-dev libavformat-dev \
    libavutil-dev libswscale-dev libeigen3-dev \
    libopencv-dev libsdl1.2-dev libusb-1.0-0-dev \
    libgles2-mesa-dev libcurl4-openssl-dev unzip
```

**Extras:**

```bash
sudo apt-get install -y \
    ros-noetic-joy ros-noetic-geometry-msgs ros-noetic-cv-bridge \
    python3-numpy python3-opencv python3-yaml
```

[🔙 Volver al Índice](#indice)

---
<a id="instalacion-desde-cero"></a>

## 🔧 Instalación desde Cero

### 1️⃣ Crear workspace y compilar

```bash
mkdir -p ~/bebop_ws/src
cd ~/bebop_ws
catkin_make
```

### 2️⃣ Configurar entorno

```bash
echo "source ~/bebop_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3️⃣ Clonar repositorios [como se indica](https://github.com/antonellabarisic/parrot_arsdk/tree/noetic_dev):

**Parrot ARSDK**

```bash
cd ~/bebop_ws/src
git clone https://github.com/antonellabarisic/parrot_arsdk.git
cd parrot_arsdk
git checkout noetic_dev
sudo apt-get install libavahi-client-dev
sudo ln -s /usr/bin/python3 /usr/bin/python
cd ~/bebop_ws
catkin_make
```

**Bebop Autonomy**

```bash
cd ~/bebop_ws/src
git clone https://github.com/AutonomyLab/bebop_autonomy.git
```

Modificar en **bebop\_driver/src/bebop\_video\_decoder.cpp**:

```
línea 93: CODEC_AP_TRUNCATED -> AV_CODEC_CAP_TRUNCATED
línea 95: CODEC_FLAG_TRUNCATED -> AV_CODEC_FLAG_TRUNCATED
línea 97: CODEC_FLAG2_CHUNKS -> AV_CODEC_FLAG2_CHUNKS
```

Añadir a `~/.bashrc`:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/bebop_ws/devel/lib/parrot_arsdk
```

### 4️⃣ Compilar todo

```bash
cd ~/bebop_ws
catkin_make -j1
source devel/setup.bash
```
[🔙 Volver al Índice](#indice)

---

<a id="uso-del-drone-parrot-bebop-2"></a>

## ▶️ Uso del Drone Parrot Bebop 2

<a id="conexion-con-el-dron"></a>

### 1️⃣ Conexión con el dron

---

#### 🔹 Conectar a la red WiFi del Bebop

Conéctate desde la configuración de tu sistema o usando la siguiente línea de comando:

```bash
nmcli dev wifi connect "Bebop2-XXXXXX"
```

> Reemplaza `"Bebop2-XXXXXX"` con el nombre exacto de la red de tu dron.

---

#### 🔹 Verificar IP y conexión de red

Después de conectarte, debes asegurarte de que tu equipo tiene la IP correcta y puede comunicarse con el dron.

1. **Comprobar la red conectada**

   ```bash
   iwconfig
   ```

   * **Para qué sirve:** Muestra la interfaz inalámbrica y la red actual.
   * **Qué deberías ver:** El nombre de la interfaz (ej. `wlo1`) y el SSID del dron (`Bebop2-XXXXXX`).

2. **Confirmar la IP asignada**

   ```bash
   ifconfig wlo1
   ```

   * **Para qué sirve:** Verifica la configuración de la interfaz WiFi.
   * **Qué deberías ver:** Una IP en el rango `192.168.42.xx`. Ejemplo:

     ```
     inet 192.168.42.22  netmask 255.255.255.0
     ```

3. **Asignar IP manualmente si no hay**

   ```bash
   sudo dhclient wlo1
   ```

   * **Para qué sirve:** Solicita una dirección IP al dron.
   * **Qué deberías ver:** Tras ejecutar de nuevo `ifconfig wlo1`, ahora aparece la IP correcta.

---

#### 🔹 Verificar conexión con ping

```bash
ping 192.168.42.1
```

Al probar la conexión, pueden ocurrir dos casos:

❌ **Respuesta incorrecta (sin conexión activa):**

```
PING 192.168.42.1 (192.168.42.1) 56(84) bytes of data.
From 192.168.42.22 icmp_seq=1 Destination Host Unreachable
^C
--- 192.168.42.1 ping statistics ---
3 packets transmitted, 0 received, +1 errors, 100% packet loss, time 2033ms
```

✅ **Respuesta correcta (conexión activa):**

```
PING 192.168.42.1 (192.168.42.1) 56(84) bytes of data.
64 bytes from 192.168.42.1: icmp_seq=1 ttl=64 time=2.76 ms
64 bytes from 192.168.42.1: icmp_seq=2 ttl=64 time=3.01 ms
...
64 bytes from 192.168.42.1: icmp_seq=27 ttl=64 time=83.4 ms
^C
--- 192.168.42.1 ping statistics ---
27 packets transmitted, 27 received, 0% packet loss, time 26050ms
rtt min/avg/max/mdev = 1.567/23.327/166.341/43.711 ms
```

💡 **Interpretación:**

* Si ves el ejemplo ✅, la conexión con el dron está activa y puedes continuar con los comandos de ROS.
* Si aparece el ejemplo ❌, revisa la conexión WiFi, la IP y asegúrate de que el dron esté encendido.

[🔙 Volver al Índice](#indice)

---

<a id="iniciar-ros"></a>

### 2️⃣ Iniciar ROS

Antes de ejecutar cualquier nodo o comando, debes iniciar el **roscore**, que es el núcleo de ROS.
`roscore` es un servicio que permite que todos los nodos y tópicos de ROS se comuniquen entre sí.

```bash
roscore
```

> Debe mantenerse ejecutando en una terminal mientras usas ROS.

[🔙 Volver al Índice](#indice)

---

<a id="lanzar-el-nodo-principal"></a>

### 3️⃣ Lanzar el Nodo Principal

El nodo principal del Bebop (`bebop_node`) controla la comunicación con el dron, recibe datos de sensores y envía comandos de vuelo.
Para iniciarlo:

```bash
roslaunch bebop_driver bebop_node.launch
```

> Este comando se ejecuta en una nueva terminal con `setup.bash` cargado.
> Una vez lanzado, el dron estará listo para recibir comandos y enviar datos a ROS.

[🔙 Volver al Índice](#indice)

---

<a id="comandos-basicos"></a>

### 4️⃣ Comandos Básicos

Esta sección te permite **controlar el dron desde la terminal** mediante `rostopic pub`, publicando mensajes en los tópicos correspondientes.

> ⚠️ **Precaución:** Antes de ejecutar cualquier comando, asegúrate de tener suficiente espacio libre alrededor del dron y que no haya obstáculos. Cada comando debe ejecutarse en una nueva terminal con el `setup.bash` cargado, mientras `roscore` y el nodo principal están corriendo.

---


#### 🔹 Diferencia entre `--once` y `-r <rate>`

> 🟢 `--once` → Movimiento **instantáneo**, solo un impulso breve.
> 🔵 `-r 10` → Movimiento **continuo**, se repite 10 veces por segundo hasta detenerlo (Ctrl+C o Detener movimiento).

---

#### 🔹 Despegar y aterrizar

| Acción    | Comando                                                  | Explicación                                                   |
| --------- | -------------------------------------------------------- | ------------------------------------------------------------- |
| Despegar  | `rostopic pub --once /bebop/takeoff std_msgs/Empty "{}"` | El dron despega y se mantiene flotando a baja altura (\~1 m). |
| Aterrizar | `rostopic pub --once /bebop/land std_msgs/Empty "{}"`    | El dron desciende suavemente hasta tocar el suelo.            |

---

#### 🔹 Movimientos Básicos del Bebop

##### 1️⃣ Avanzar / Retroceder

* **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.2, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

Avanza solo un instante (\~unos centímetros).

* **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.2, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

Avanza continuamente a 0.2 m/s hasta que presiones Ctrl+C o publiques **Detener movimiento**.

* Retroceder: cambia `x` a negativo (`x:-0.2`).

---

##### 2️⃣ Giros (izquierda / derecha)

* **🟢 Instantáneo:**

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.3}}'
```

* **🔵 Continuo:**

```bash
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.3}}'
```

* Girar a la derecha: cambia `z` a negativo (`z:-0.3`).

---

##### 3️⃣ Subir / Bajar

* **🟢 Instantáneo:**

```bash
# Subir
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.2}, angular: {x:0.0, y:0.0, z:0.0}}'

# Bajar
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:-0.2}, angular: {x:0.0, y:0.0, z:0.0}}'
```

* **🔵 Continuo:**

```bash
# Subir
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.2}, angular: {x:0.0, y:0.0, z:0.0}}'

# Bajar
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:-0.2}, angular: {x:0.0, y:0.0, z:0.0}}'
```

---

##### 4️⃣ Movimiento lateral (izquierda / derecha)

* **🟢 Instantáneo:**

```bash
# Derecha
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.2, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'

# Izquierda
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:-0.2, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

* **🔵 Continuo:**

```bash
# Derecha
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.2, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'

# Izquierda
rostopic pub -r 10 /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:-0.2, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

---

#### 🔹 Detener o emergencia

* **Detener movimiento:** Frenar inmediatamente cualquier movimiento continuo:

```bash
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist \
'{linear: {x:0.0, y:0.0, z:0.0}, angular: {x:0.0, y:0.0, z:0.0}}'
```

* **Emergencia:** Apaga los motores de inmediato y reinicia los sistemas del dron.

```bash
rostopic pub --once /bebop/reset std_msgs/Empty "{}"
```

---

> ⚠️ **Recomendaciones generales:**
>
> * Ajusta los valores `x`, `y`, `z` para controlar velocidad y dirección.
> * Mantén siempre suficiente espacio alrededor del dron antes de ejecutar comandos de vuelo.
> * Para detener cualquier movimiento continuo, puedes usar **Ctrl+C** o publicar un **Detener movimiento**.
> * Para situaciones de riesgo o emergencia, usa **`/bebop/reset`** para **apagar los motores de inmediato**.

[🔙 Volver al Índice](#indice)

---

<a id="verificar-topicos-disponibles"></a>

### 5️⃣ Verificar Tópicos Disponibles

Los **tópicos** son canales de comunicación entre nodos de ROS.
Esta sección permite **ver qué información envía y recibe el dron**, como la cámara, la odometría o la batería.

```bash
rostopic list | grep bebop
```

> Esto mostrará todos los tópicos relacionados con el dron.
> Para ver la información de un tópico en tiempo real, usa `rostopic echo <nombre_del_topic>`.

Principales:

```
/bebop/cmd_vel
/bebop/takeoff
/bebop/land
/bebop/reset
/bebop/image_raw
/bebop/odom
/bebop/imu/data
/bebop/states/common/CommonState/BatteryStateChanged
```

Ver datos de batería:

```bash
rostopic echo /bebop/states/common/CommonState/BatteryStateChanged
```

[🔙 Volver al Índice](#indice)

---

<a id="ver-la-camara"></a>

### 6️⃣ Ver la Cámara

Permite **visualizar la cámara delantera del dron en tiempo real**.
Se usa `rqt_image_view` para abrir una ventana donde se muestra el video:

```bash
rqt_image_view /bebop/image_raw
```

> Esto es útil para inspeccionar el entorno o hacer pruebas de visión por computadora.

[🔙 Volver al Índice](#indice)

---

<a id="visualizar-nodos-y-topicos-rqt-graph"></a>

### 7️⃣ Visualizar Nodos y Tópicos (`rqt_graph`)

`rqt_graph` muestra un **diagrama visual de los nodos y sus conexiones** en ROS.
Esto te ayuda a entender cómo se comunican los distintos componentes del dron, por ejemplo:

* Qué nodo envía comandos a los motores (`cmd_vel`)
* Qué nodo publica las imágenes de la cámara
* Qué nodo informa el estado de la batería o la odometría

```bash
rqt_graph
```

> Ideal para depurar problemas o entender la arquitectura de ROS si eres nuevo en el sistema.

Ejemplo de flujo básico en Bebop:

```
         +-------------+
         |bebop_node   |
         +-------------+
          /     |      \
     cmd_vel  camera    state
       |        |         |
   [motores]  [video]   [info]
```
[🔙 Volver al Índice](#indice)

---

<a id="ejemplo-python-vuelo-simple"></a>

### 8️⃣ Ejemplo Python - Vuelo Simple

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

rospy.init_node('bebop_test_flight')

takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
cmd_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

time.sleep(1)

# Despegar
takeoff_pub.publish(Empty())
rospy.loginfo("Despegando...")
time.sleep(5)

# Avanzar
move = Twist()
move.linear.x = 0.2
cmd_pub.publish(move)
time.sleep(3)

# Girar a la izquierda
move = Twist()
move.angular.z = 0.3
cmd_pub.publish(move)
time.sleep(3)

cmd_pub.publish(Twist())  # Detener
land_pub.publish(Empty())
rospy.loginfo("Aterrizando...")
time.sleep(5)
```
[🔙 Volver al Índice](#indice)

---

<a id="diagrama-basico-del-flujo-de-vuelo"></a>

### 9️⃣ Diagrama Básico del Flujo de Vuelo

```
[ TAKEOFF ] → [ CMD_VEL (mover/girar) ] → [ LAND ]
```

---

