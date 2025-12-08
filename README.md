# slave_init_bo
He hagut de fer uns canvis al CMake del CanOpenNode. Si només us voleu descarregar el main, primer mireu que us quadrin els CMake. El que he fet és accedir als fitxers d'exemple. 

Potser en un futur caldrà treure aquests fitxers d'exemple de dins de la carpeta exemples. 

L'slave es on es configura l'adressa LSS única per després poder fer l'assignació automàtica dels nodes
Hi ha l'inicialització de la memòria volàtil. Això és perquè hi hagi paràmetres del node que es guardin encara que es faci un reset. 
Quan un node ja està configurat, no entrarà a la parafernalia de reconfigurar-se, sinó que ja tindrà tot a lloc i esperarà el NMT operational command. 

# Diagrames i flux de l'applicació

1. La funció `CO_ESP32_LSS_Run()` se executa desde main.c - Configura pins i executa la tasca `CO_mainTask`
2. **MainTask**:
	1. Crea instància `CO` del stack *CANopen*
	2. S'inicialitza el bus
	3. S'inicialitza LSS amb una direcció única derivada de la MAC
	4. S'inicialitza el stack CANopen complet
	5. Es crea la tasca periòdica `CO_periodicTask`
3. **PeriodicTask** - Logica cíclica del usuari
	1. Es procesen RPDO i TPDO
	2. S'envia un contador dummy (versió actual)
	3. Gestió d'emergències
		1. Si esta polsat el botó -> es reporta un error CANopen (EMCY)
		2. Es neteja l'error
		3. S'actualitza el OD y es mostra

  
<img width="375" height="1440" alt="image" src="https://github.com/user-attachments/assets/b725ca51-20f0-43b3-b4a0-ddc431be63ca" />

### Diagrama 2
<img width="740" height="592" alt="Pasted image 20251208125606" src="https://github.com/user-attachments/assets/a3938c48-89e6-455c-b07e-f180f933887d" />
