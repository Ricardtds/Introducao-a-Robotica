# ROTEIRO DE INSTALAÇÃO DO TURTLEBOT2

## Instalando

* Ubuntu 20.04 em máquina virtual
* ROS1 versão NOETIC
* TurtleBot2

---

## 1 – PASSO – Baixar a imagem do Ubuntu 20.04

Disponível no link:
[https://releases.ubuntu.com/focal/](https://releases.ubuntu.com/focal/)

---

## 2 – PASSO – Baixar e instalar o software VirtualBox

Você pode baixar o VirtualBox no site:
[https://www.virtualbox.org/wiki/Downloads](https://www.virtualbox.org/wiki/Downloads)

Siga as instruções para instalar no seu sistema operacional.
Após a instalação, inicie o VirtualBox.

---

## 3 – PASSO – Instalando e criando uma máquina virtual

Clique em **NEW** para criar uma máquina virtual e preencha os campos:

* **Name:** Nome da máquina virtual
* **Machine Folder:** Local onde o SO será instalado
* **ISO Image:** Caminho para a ISO do Ubuntu 20.04

Preencha também:

* Nome de usuário
* Senha
* Nome da máquina (sem espaços)

Configure os recursos da VM:

* **Base Memory:** RAM
* **Processors:** Número de núcleos

Defina o espaço em disco (mínimo recomendado: **25 GB**).
Clique em **Next** e **Finish** para finalizar.

---

## 4 – PASSO – Inicializando o Ubuntu

> ⚠️ **NÃO FAÇA UPDATE PARA A VERSÃO 22!**

Abra o terminal:

### Activities → Terminal

Entre como superusuário:

```bash
su root
```

Configure o teclado para abnt2 caso o seu teclado seja assim:

```bash
setxkbmap -model abnt2 -layout br
```

Dê privilégios de `sudo` ao usuário que você criou:

```bash
usermod -aG sudo user_name
```

reinicie a máquina:

```bash
reboot
```

---

## 5 – PASSO – Instalação do ROS1 versão Noetic

``` bash
wget -qO - https://raw.githubusercontent.com/Ricardtds/Introducao-a-Robotica/refs/heads/main/ros/ros-install.sh | bash
```
