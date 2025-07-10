📝 Descrição

Este pacote ROS2 fornece um nó para controle teleoperado de robôs móveis com rodas usando um joystick. O controle permite movimento linear (frente/trás) e angular (esquerda/direita), com freio de emergência e mensagens informativas no terminal.
🎮 Funcionalidades

    Controle de velocidade linear e angular via joystick
    
📦 Dependências

    ROS2 Humble (ou versão mais recente)

    sensor_msgs

    geometry_msgs

    rclcpp

🛠️ Instalação

    Clone este repositório para o seu workspace ROS2:

bash

cd ~/ros2_ws/src📝 Descrição

Este pacote ROS2 fornece um nó para controle teleoperado de robôs móveis com rodas usando um joystick. O controle permite movimento linear (frente/trás) e angular (esquerda/direita), com freio de emergência e mensagens informativas no terminal.
🎮 Funcionalidades

    Controle de velocidade linear e angular via joystick

    Botão de segurança (enable) para habilitar o controle

    Freio de emergência com mensagem destacada

    Mensagens informativas para cada direção de movimento

    Parâmetros configuráveis via arquivo YAML

📦 Dependências

    ROS2 Humble (ou versão mais recente)

    sensor_msgs

    geometry_msgs

    rclcpp

🛠️ Instalação

    Clone este repositório para o seu workspace ROS2:

bash

cd ~/ros2_ws/src
git clone https://github.com/emmano2997/ros2_teleop_ws.git

    Construa o pacote:

bash

cd ~/ros2_ws
colcon build --packages-select teleop_controller
source install/setup.bash
git clone https://github.com/emmano2997/ros2_teleop_ws.git

    Construa o pacote:

bash

cd ~/ros2_ws
colcon build --packages-select teleop_controller
source install/setup.bash


ros2 run joy joy_node
