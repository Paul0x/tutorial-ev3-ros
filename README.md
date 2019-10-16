# Guia de Instalação e Inicialização do Ev3 Utilizando h4r-ev3 e Wi-Fi

## 1. Instalação do h4r

## 2. Inicialização
### 2.1. Pré-Requisitos
- Ev3 com h4r-ev3 instalado
- Cabo de conexão USB para o Ev3 (USB -> A-Mini-USB)
- Teclado USB
- Adaptador Wireless com driver configurado no ev3
- Computador com linux instalado

### 2.2. Inicializando e Acessando o Ev3 via Wi-Fi

1. Ligue o Ev3 e aguarde até sua luz ficar verde e aparecer a tela de login;

2. Insira o teclado USB e conecte o cabo de conexão no Ev3 e no PC;

3. Faça o login digitando no console do Ev3:
  ```console
root@ev3:~$ root 
```
4. Inicialize a placa de rede virtual do Ev3:
  ```console
root@ev3:~$ gadget_on 
```
5. Agora deve ser possível se conectar à rede do Ev3 através do cabo USB. Feito isso, é possível acessar o Ev3 pelo computador.

6. Ao conectar na rede do Ev3, é necessário configurar seu IP manualmente para *192.168.10.<?>*, onde *<?>* é qualquer número entre 0-200 e diferente de 123.

7. Feito isso, digite no **terminal do computador**:

  ```console
usuario@pc:~$ ssh root@192.168.10.123 
root@ev3:~$ login: root

```
8. Agora vamos habilitar o Wi-Fi do Ev3, digite no **terminal conectado via ssh**:

  ```console
root@ev3:~$ connmanctl # Abre o console do connman
root@ev3:~$ enable wifi # Habilita o Wi-Fi
root@ev3:~$ scan wifi # Procura as redes wifi
root@ev3:~$ services # Mostra as redes disponíveis
root@ev3:~$ connect wifi_(HASH) # Onde (HASH) é o código de sua rede, exibida pelo comando anterior

```

9. Se tudo deu certo, seu Ev3 deve estar conectado via Wi-Fi e você não conseguirá mais mexer no terminal aberto via ssh.

> **NOTA:** Caso ocorra algum problema, principalmente no comando *enable wifi*, verifique se o driver do seu adaptador está instalado no Ev3 (SESSÃO 1.X)

### 2.3. Configurando Sua Rede Wifi com os Seus Dispositivos
#### 2.3.1. Configurando os IPs no Roteador
#### 2.3.2. Configurando Arquivo HOSTS

### 2.4. Inicializando o ROS e os Controllers do Ev3
#### 2.4.1. Inicializando o ROS
#### 2.4.2. Inicializando os Controllers
#### 2.4.3. Movimentando o Ev3

## 2.5. Controlando o Ev3 Utilizando RosPy





  
