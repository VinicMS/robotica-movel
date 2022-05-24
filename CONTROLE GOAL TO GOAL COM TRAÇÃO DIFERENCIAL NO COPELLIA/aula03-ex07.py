"""
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Móvel - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Data: 08/05/2022
 
 # Modificação dos exemplos para controle direto da IDE Python
 # Utilize o PLAY e o CTRL+C do Spyder IDE para controlar a simulação sem 
   necessidade de clicar nos botões do CoppeliaSim
"""

import vrep
import time
import sys
import numpy as np

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
   print ('Servidor conectado!') 
else:
    print ('Problemas para conectar o servidor!')
    sys.exit()

#Ativa modo síncrono da RemoteAPI
vrep.simxSynchronous(clientID, True) 

#Inicia a simulação
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

# Handles da Cena [Todos os Objetos que compõem o robô]
_, TargetbodyHandle = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)
_, leftMotorHandle = vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_oneshot_wait)
_, rightMotorHandle = vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_oneshot_wait)
_, bodyHandle = vrep.simxGetObjectHandle(clientID,'Robo_Ifes',vrep.simx_opmode_oneshot_wait)

#Parâmetros do robô

R = 0.035   # Raio da Roda
L = 0.28    # Distância entre-rodas 

#Função de Mapeamento do Uniciclo (v,w) para rodas (vr, vl)
def Controlar_Motores_Uniciclo(u,w):    
    wd=(2*u+w*L)/(2*R)
    we=(2*u-w*L)/(2*R)
    
    vrep.simxSetJointTargetVelocity(clientID,  leftMotorHandle, we, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, wd, vrep.simx_opmode_streaming)
     
#Função que simula algoritmos globais de sensoriamento [Ex. Visão Computacional]
def Obter_Posicao(handle): 
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1,vrep.simx_opmode_streaming)
    _, ori_body = vrep.simxGetObjectOrientation(clientID, handle,-1,vrep.simx_opmode_streaming)

    phi = ori_body[2] 
    phi = np.arctan2(np.sin(phi), np.cos(phi)) #[rad] - Correção domínio [-pi, pi]
    x = pos[0]   
    y = pos[1]
    
    return x, y, phi
     
def main():    
    #Variáveis  de Simulação (Parâmetros medidos/estabelecidos da cena do V-REP)
    t = 0       # Guarda tempo de simulação no Python
    dt = 0.05   # Intervalo de integração
    
    #Loop de controle do robô
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter()
        
        t += dt
        
        xg, yg, phig = Obter_Posicao(TargetbodyHandle)     
        x, y, phi = Obter_Posicao(bodyHandle)
        
        ###################################################################
        #### Implementar Controlador Goal-To-Goal do Robô aqui     ########
        ###################################################################
        
        #orientação phi (heading) até o ponto desejado
        phid = np.arctan2(yg-y,xg-x)
        #toma o erro com relacao a orientacao atual
        e = phid - phi
        #normaliza em 360 (ou 2pi)
        e = Norm_Ang(e) 
        #rho calcula a distancia ate o ponto desejado
        rho = np.sqrt((xg-x)**2 + (yg-y)**2)
        
        #testar se rho<l, se sim, zera tudo e para de mover.
        if rho < tolerancia :
            chave = False #chave falsa, zera tudo (u e w).
            #count+=1
        else:
            chave=True
        
        if chave == True:
            u = u0
            w = Kp * e
        else:
            u = 0.0
            w = 0.0
        
        Controlar_Motores_Uniciclo(u,w) #Movimento de exemplo    
        ###################################################################
        #### ##############################################################
        ###################################################################
        
        print("xg: %1.2f  yg: %1.2f  phig: %1.2f" % (xg, yg, phig))
        print("x : %1.2f  y : %1.2f  phi : %1.2f" % (x, y, phi))
        print("Tempo de Simulação: %1.2f [seg]" % (t))
        
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        
        while(time.perf_counter()-t0 <= dt): _ # Para o loop até 50ms

try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)