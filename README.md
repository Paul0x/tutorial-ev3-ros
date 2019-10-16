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

#### 2.2.1. Inicializando o Ev3
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
```




  
