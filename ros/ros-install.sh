#!/bin/bash

set -e
trap 'echo "❌ Erro na linha $LINENO. Abortando..."; exit 1' ERR

# Valida a senha uma vez
echo "🔐 Solicitando autenticação sudo..."
sudo -v

# Mantém sudo ativo durante todo o script
( while true; do sudo -n true; sleep 60; done ) &

# Salva o PID do processo de manutenção
SUDO_KEEPALIVE_PID=$!

# Função para matar o processo ao sair do script
cleanup() {
  echo "🧹 Limpando..."
  kill "$SUDO_KEEPALIVE_PID"
}
trap cleanup EXIT

# Sair em caso de erro
set -e

echo ">>> Atualizando pacotes..."
sudo apt update && sudo apt upgrade -y

echo ">>> Instalando curl e git..."
sudo apt install -y curl git python3-pip

echo ">>> Adicionando repositório do ROS Noetic..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

echo ">>> Adicionando chave do ROS..."
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo ">>> Instalando ROS Noetic Desktop Full..."
sudo apt update
sudo apt install -y ros-noetic-desktop-full

sleep 2

echo ">>> Configurando ambiente ROS..."
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo ">>> Instalando dependências para o workspace Catkin..."
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sleep 2

echo ">>> Inicializando rosdep..."
sudo rosdep init || true
rosdep update

source ~/.bashrc

echo ">>> Criando workspace Catkin..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

echo ">>> Baixando script de instalação básica do Turtlebot2..."
curl -sLf https://raw.githubusercontent.com/Thxssio/Turtlebot2-On-noetic/master/install_basic.sh | bash

echo ">>> Compilando o workspace Catkin..."
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
sleep 2
catkin_make


echo ">>> Configurando ambiente do workspace Catkin..."
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo ">>> Corrigindo possível erro do Python..."
sudo ln -sf /usr/bin/python3 /usr/bin/python || true
pip install -U rospy
sudo apt install -y python3-rospy python3-tf2-ros python3-nav-msgs python3-geometry-msgs python3-tf python3-tf2 python3-heapdict python3-numpy python3-scipy

echo ">>> Instalação concluída com sucesso!"