"""
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Móvel - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Revisado em: 22/05/2022
 
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

#Chama função genérica de reset do encoder. Está no Child Script das rodas no simulador V-REP
vrep.simxCallScriptFunction(clientID,'RightWheel',vrep.sim_scripttype_childscript,'Reset_Right_Encoder',[],[],[],bytearray(),vrep.simx_opmode_blocking)
vrep.simxCallScriptFunction(clientID,'LeftWheel',vrep.sim_scripttype_childscript,'Reset_Left_Encoder',[],[],[],bytearray(),vrep.simx_opmode_blocking)

R = 0.033   # Raio da Roda
L = 0.31    # Distância entre-rodas
    
#Função de Controle do Uniciclo a partir de (v,w)
def ControleUniciclo(u,w):
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

#Função que captura pulsos de encoders simulados nas rodas do robô
#Encoder está simulado no Child Script de cada roda na cena do V-REP 
def getEncoderPulses():
    _, pe = vrep.simxGetIntegerSignal(clientID,'Left_Encoder',vrep.simx_opmode_streaming)
    _, pd = vrep.simxGetIntegerSignal(clientID,'Right_Encoder',vrep.simx_opmode_streaming)
     
    return pe, pd
  
def norm_ang(a):
    return np.arctan2(np.sin(a),np.cos(a))

def main():    
    #Variáveis 
    t = 0       # Guarda tempo de simulação no Python
    dt = 0.05   # Intervalo de integração
    N = 20      # Resolução do Encoder
    pd_ant = 0  # Auxiliar - Cáluculo da variação de pulsos
    pe_ant = 0  # Auxiliar - Cáluculo da variação de pulsos
    pe = 0
    pd = 0
    dpe = 0
    dpd = 0
    Kp_g = 1
    u_max = 12

    erro_E = 0
    erro_old_E = 0
    erro_i_E = 0
    erro_d_E = 0
    erro_D = 0
    erro_old_D = 0
    erro_i_D = 0
    erro_d_D = 0

    Kp_E = 0.15
    Ki_E = 0
    Kd_E = 0

    Kp_D = 0.15
    Ki_D = 0
    Kd_D = 0

    u0 = 0.5

    phi_o = 0
    x_o = 0
    y_o = 0
    
    #Loop de controle do robô
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter()
        
        t += dt
        
        #Controle de posição   
        xg,     yg,     phig = Obter_Posicao(TargetbodyHandle)   
        x_real, y_real, phi_real = Obter_Posicao(bodyHandle)

        #orientação phi (heading) até o ponto desejado
        phid_o = np.arctan2(yg-y_o,xg-x_o)
        #toma o erro com relacao a orientacao atual
        e_o = phid_o - phi_o
        #normaliza em 360 (ou 2pi)
        e_o = np.arctan2(np.sin(e_o), np.cos(e_o))
        #rho calcula a distancia ate o ponto desejado
        rho_o = np.sqrt((xg-x_o)**2 + (yg-y_o)**2)

        #testar se rho<l, se sim, zera tudo e para de mover.
        if rho_o < 0.1 :
            chave = False #chave falsa, zera tudo (u e w).
            #count+=1
        else:
            chave=True

        if chave == True:
            u = u0
            w = Kp_g * e_o
        else:
            u = 0.0
            w = 0.0

        #Obtém pulsos do encoder (pl- pulsos da esquerda) (pr - pulsos da direita)
        pe, pd = getEncoderPulses()
        
        dpe = (pe - pe_ant)/dt
        dpd = (pd - pd_ant)/dt

        we_pulso = (dpe*2*np.pi)/N
        wd_pulso = (dpd*2*np.pi)/N
        pe_ant, pd_ant = pe, pd

        we_o = ((2*u - w*L)/(2*R))
        wd_o = ((2*u + w*L)/(2*R))

        #ODOMETRIA COM INTEGRAÇÃO NUMÉRICA
        phi_p_o = (R/L)*(wd_pulso - we_pulso)
        x_p_o = (R/2)*(wd_pulso + we_pulso)*np.cos(phi_o)
        y_p_o = (R/2)*(wd_pulso + we_pulso)*np.sin(phi_o)
        
        phi_o = phi_o + phi_p_o*dt
        x_o = x_o + x_p_o*dt
        y_o = y_o + y_p_o*dt

        u_E = 0
        u_D = 0


        #ESQUERDA

        erro_E = we_o - we_pulso
        erro_d_E = (erro_E - erro_old_E)/dt
        erro_i_E = erro_i_E + erro_E*dt
        erro_old_E = erro_E

        u_E = Kp_E*erro_E + Ki_E*erro_i_E + Kd_E*erro_d_E

        if abs(u_E)>12:
            if u_E>0:
                u_E = u_max
            else:
                u_E = -u_max

        erro_D = wd_o - wd_pulso
        erro_d_D = (erro_D - erro_old_D)/dt
        erro_i_D = erro_i_D + erro_D*dt
        erro_old_D = erro_D

        u_D = Kp_D*erro_D + Ki_D*erro_i_D + Kd_D*erro_d_D
        if abs(u_D)>12:
            if u_D>0:
                u_D = u_max
            else:
                u_D = -u_max   
        
        vrep.simxSetJointTargetVelocity(clientID,  leftMotorHandle, u_E, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, u_D, vrep.simx_opmode_streaming)
        
        print("Pose Real        (x,y,phi): (%1.2f, %1.2f, %1.2f)" % (x_real, y_real, phi_real))
        print("Pose Odometria   (x,y,phi): (%1.2f, %1.2f, %1.2f)" % (x_o, y_o, phi_o))
        #print("Pose do Alvo     (x,y,phi): (%1.2f, %1.2f, %1.2f)" % (xg, yg, phig))
        #print("Pulsos dos Encoders (E, D): (%d, %d)" % (pe,pd))
        #print("we, wd: (%d, %d)" % (wl,wr))
        
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        
        while(time.perf_counter()-t0 <= dt): _ # Para o loop até 50ms
    

try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)