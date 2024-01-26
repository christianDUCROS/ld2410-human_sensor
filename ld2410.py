# HLK-LD2410 B et C  (Microwave-based human/object presence sensor) 
#rev 1 DUCROS christian janvier 2024 à partir du fichier 
#rev 1 - shabaz - May 2023
import  utime

#------------ 2.1 Command protocol frame format---------------
HEADER = bytes([0xfd, 0xfc, 0xfb, 0xfa])
TERMINATOR = bytes([0x04, 0x03, 0x02, 0x01])
#----------- 2.3 Reported data frame format-------------------
# Example data report in Basic mode:
# hex: f4 f3 f2 f1 0d 00 02 aa 03 4f 00 64 4c 00 64 32 00 55 00 f8 f7 f6 f5
REPORT_HEADER = bytes([0xf4, 0xf3, 0xf2, 0xf1])
# bytes 0-3 are the header, always 0xf4, 0xf3, 0xf2, 0xf1
# bytes 4-5 are the frame length, always 0x0d, 0x00 for Basic mode
# byte 6 is the report type, always 0x02 for Basic mode, or 0x01 for Engineering mode
# byte 7 is the report head, always 0xaa
# byte 8 is the state, 0x00 = no target, 0x01 = moving target, 0x02 = stationary target, 0x03 = combined target
# bytes 9-10 are the moving target distance in cm, little endian
# byte 11 is the moving target energy
# bytes 12-13 are the stationary target distance in cm, little endian
# byte 14 is the stationary target energy
# bytes 15-16 are the detection distance in cm, little endian
REPORT_TERMINATOR = bytes([0xf8, 0xf7, 0xf6, 0xf5])

NULLDATA = bytes([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]) #no response --> ack = 0 donc failure

STATE_NO_TARGET = 0
STATE_MOVING_TARGET = 1
STATE_STATIONARY_TARGET = 2
STATE_COMBINED_TARGET = 3
TARGET_NAME = ["no_target", "moving_target", "stationary_target", "combined_target"]



class LD2410() :
    
    def __init__(self,bus_uart): 
        self.ser = bus_uart
        
        self.meas = {
                "state": STATE_NO_TARGET,
                "moving_distance": 0,
                "moving_energy": 0,
                "stationary_distance": 0,
                "stationary_energy": 0,
                "detection_distance": 0 }
        
        self.communication_error = 0 

    #---------fonctions communes configuration ----------
    #affichage des trames - Utiliser pour debugger
    def print_trames_bytes(self,data):  
        try :                      
            text = f"hex: "
            for i in range(0, len(data)):
                text = text + f" {data[i]:02x}"
            print(text)
        except :
            pass
    #flush : vidange du buffer (lecture des datas)
    def serial_flush(self):
        dummy = self.ser.read()
        #return dummy

     #----------------Send command with ACK------------------------------------
    def send_command(self, cmd_values):
        cmd_data_len = bytes([len(cmd_values), 0x00]) #little endian
        frame = HEADER + cmd_data_len + cmd_values + TERMINATOR
        self.ser.write(frame)
        #print(frame)#debug
        #self.serial_flush() 
        utime.sleep_ms(20)
        #----------------reception  message----------------
        if self.ser.any() > 0: 
            #Lire le message reçu
            response = self.ser.read()
            #self.print_trames_bytes(response) # debug
            if len(response) <10 :
                response = NULLDATA
            return response
        else :
            print("probleme communication : reponse vide ")
            response = NULLDATA
            return response
 
    #2.2.1 
    def enable_config(self):
        cmd = 0x00FF
        value = 0x0001
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        octet1_value = (value & 0xFF00) >> 8
        octet2_value = value & 0x00FF
        # Créer la séquence d'octets en ordre spécifié
        cmd_value = bytes([octet2_cmd, octet1_cmd, octet2_value, octet1_value])
        response = self.send_command(cmd_value)
        if response[7] :
            print('enable config success')
            return 1
        else :
            print('enable config failure')
            return 0
        
    #2.2.2
    def end_config(self):
        cmd = 0x00FE
        value = None
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        cmd_value = bytes([octet2_cmd, octet1_cmd])
        response = self.send_command(cmd_value)
        if response[7] :
            print('end config success')
            return 1
        else :
            print('end config failure')
            return 0
        
     
    #2.2.3
    def Maximum_distance_gate_and_unoccupied_duration_parameters_configuration(self,maximum_mouvement_distance_door=8,maximum_resting_distance_door=8,no_one_duration=5):
        cmd = 0x0060
        param0 = 0x0000
        param1 = 0x0001
        param2 = 0x0002 
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        octet1_param0 = (param0 & 0xFF00) >> 8
        octet2_param0 = param0 & 0x00FF
        octet1_param1 = (param1 & 0xFF00) >> 8
        octet2_param1 = param1 & 0x00FF
        octet1_param2 = (param2 & 0xFF00) >> 8
        octet2_param2 = param2 & 0x00FF
        maximum_mouvement_distance_door = maximum_mouvement_distance_door.to_bytes(4,'little')
        maximum_resting_distance_door = maximum_resting_distance_door.to_bytes(4,'little')
        no_one_duration = no_one_duration.to_bytes(4,'little')
        cmd_values = bytes([octet2_cmd, octet1_cmd,octet2_param0,octet1_param0])+maximum_mouvement_distance_door+bytes([octet2_param1,octet1_param1])+maximum_resting_distance_door+bytes([octet2_param2,octet1_param2])+no_one_duration
        #cmd_values = cmd + param1 + maximum_mouvement_distance_door + param2+maximum_resting_distance_door + param3 + no_one_duration
        response = self.send_command(cmd_values)
        if response[7] and response[4] == 0x04 :
            print('Maximum_distance_gate_and_unoccupied_duration_parameters_configuration success')
            return 1
        else :
            print('Maximum_distance_gate_and_unoccupied_duration_parameters_configuration failure')
            return 0
        
    #2.2.4 # A finir parser
    def read_parameter(self):
        cmd = 0x0061
        value = None
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        cmd_value = bytes([octet2_cmd, octet1_cmd])
        response = self.send_command(cmd_value)
        self.print_trames_bytes(response)
        if response[7] :
            print('read parameter success')
            return response
        else :
            print('read parameter failure')
            return 0   
    #2.2.5    
    def enable_engineering_mode(self): 
        cmd = 0x0062
        value = None
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        cmd_value = bytes([octet2_cmd, octet1_cmd])
        response = self.send_command(cmd_value)
        if response[7] :
            print('enable engineering mode success')
            return 1
        else :
            print('enable engineering mode failure')
            return 0

    #2.2.6
    def end_engineering_mode(self): #close project mode 
        cmd = 0x0063
        value = None
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        cmd_value = bytes([octet2_cmd, octet1_cmd])
        response = self.send_command(cmd_value)
        if response[7] :
            print('enable engineering mode success ')
            return 1
        else :
            print('enable engineering mode failure')
            return 0
        
    #2.2.7
    def distance_gate_sensitivity_configuration(self,distance_gate=3,motion_sensitivity_value=40,standstill_sensitivity_value=40):
        cmd = 0x0064
        param0 = 0x0000
        param1 = 0x0001 
        param2 = 0x0002 
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        octet1_param0 = (param0 & 0xFF00) >> 8
        octet2_param0 = param0 & 0x00FF
        octet1_param1 = (param1 & 0xFF00) >> 8
        octet2_param1 = param1 & 0x00FF
        octet1_param2 = (param2 & 0xFF00) >> 8
        octet2_param2 = param2 & 0x00FF
        distance_gate = distance_gate.to_bytes(4,'little')
        motion_sensitivity_value = motion_sensitivity_value.to_bytes(4,'little')
        standstill_sensitivity_value = standstill_sensitivity_value.to_bytes(4,'little')
        cmd_values = bytes([octet2_cmd, octet1_cmd,octet2_param0,octet1_param0])+distance_gate+bytes([octet2_param1,octet1_param1])+motion_sensitivity_value+bytes([octet2_param2,octet1_param2])+standstill_sensitivity_value
        response = self.send_command(cmd_values)
        if response[7] :
            print('distance_gate_sensitivity_configuration success')
            return 1
        else :
            print('distance_gate_sensitivity_configuration failure')
            return 0
          
    #2.2.8
    def read_firmware_version(self):
        cmd = 0x00A0
        value = None
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        cmd_value = bytes([octet2_cmd, octet1_cmd])
        response = self.send_command(cmd_value)
        if response[7] :
            response_12 = hex(response[12])[2:]
            response_13 = hex(response[13])[2:]
            response_14 = hex(response[14])[2:]
            response_15 = hex(response[15])[2:]
            response_16 = hex(response[16])[2:]
            response_17 = hex(response[17])[2:]
            print('firmware :V',response_13,'.',response_12,'.',response_17,response_16,response_15,response_14)
        else :
            print('read firmware version failure')
            return 0
        
    #2.2.9
    def set_serial_port_baud_rate(self,baudrate=0x0007) : 
        cmd = 0x00A1
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        octet1_baudrate = (baudrate & 0xFF00) >> 8
        octet2_baudrate = baudrate & 0x00FF
        # Créer la séquence d'octets en ordre spécifié
        cmd_value = bytes([octet2_cmd, octet1_cmd, octet2_baudrate, octet1_baudrate])
        response = self.send_command(cmd_value)
        if response[7]  :
            print('set_serial_port_baud_rate success, baudrate : ',f"0x{baudrate:04x}" )
            return 1
        else :
            print('Set serial port baudrate failure')
            return 0    
            
    #2.2.10
    def restore_factory_settings(self):
        cmd = 0x00A2
        value = None
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        cmd_value = bytes([octet2_cmd, octet1_cmd])
        response = self.send_command(cmd_value)
        if response[7] :
            print('restore_factory_settings success')
            return 1
        else :
            print('restore_factory_settings failure')
            return 0
               
    #2.2.11
    def reboot_module(self): 
        cmd = 0x00A3
        value = None
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        cmd_value = bytes([octet2_cmd, octet1_cmd])
        response = self.send_command(cmd_value)
        if response[7] :
            print('reboot module success')
            return 1
        else :
            print('reboot module failure')
            return 0
                
    #2.2.12
    def bluetooth_setting(self,on_off=0x0001) : 
        cmd = 0x00A4
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        octet1_on_off = (on_off & 0xFF00) >> 8
        octet2_on_off= on_off & 0x00FF
        # Créer la séquence d'octets en ordre spécifié
        cmd_value = bytes([octet2_cmd, octet1_cmd, octet2_on_off, octet1_on_off])
        response = self.send_command(cmd_value)
        if response[7] :
            print('bluetooth_setting, on(1)/OFF(0) : ', on_off)
            return 1
        else :
            print('bluetooth_setting failure')
            return 0
    
    #2.2.13
    def get_mac_address(self) : 
        cmd = 0x00A5
        value = 0x0001
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        octet1_value = (value & 0xFF00) >> 8
        octet2_value= value & 0x00FF
        # Créer la séquence d'octets en ordre spécifié
        cmd_value = bytes([octet2_cmd, octet1_cmd, octet2_value, octet1_value])
        response = self.send_command(cmd_value)
        if response[7] :
            response_10 = hex(response[10])[2:]
            response_11 = hex(response[11])[2:]
            response_12 = hex(response[12])[2:]
            response_13 = hex(response[13])[2:]
            response_14 = hex(response[14])[2:]
            response_15 = hex(response[15])[2:]
            print('MAc Address',response_10,response_11,response_12,response_13,response_14,response_15)
            return response[10:16]
        else :
            print('get_mac_address failure')
            return 0
    
    #2.2.14
    def obtaining_bluetooth_permissions(self,password = 0x48694C696E6B4869): #Hilink
        cmd = 0x00A8
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        # Convertir le password hexadécimal en une chaîne hexadécimale sans le préfixe '0x'
        password_string = hex(password)[2:]
        # Assurer que la longueur de la chaîne hexadécimale est un multiple de 2
        if len(password_string) % 2 != 0:
            password_string = '0' + password_string
        # Séparer la chaîne hexadécimale en blocs de 2 caractères et convertir chaque bloc en entier hexadécimal
        password_octets = [int(password_string[i:i+2], 16) for i in range(0, len(password_string), 2)]
        cmd_values =bytes([octet2_cmd, octet1_cmd,password_octets[0],password_octets[1],password_octets[2],password_octets[3],password_octets[4],password_octets[5]])
        print (cmd_values)
        response = self.send_command(cmd_values)
        if response[7] and response != NULLDATA :
            print('obtaining_bluetooth_permissions success')
            return 1
        else :
            print('obtaining_bluetooth_permissions failure')
            return 0
    

    #2.2.15
    def set_bluetooth_password(self,password = 0x48694C696E6B): #Hilink
        cmd = 0x00A9
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        # Convertir le password hexadécimal en une chaîne hexadécimale sans le préfixe '0x'
        password_string = hex(password)[2:]
        # Assurer que la longueur de la chaîne hexadécimale est un multiple de 2
        if len(password_string) % 2 != 0:
            password_string = '0' + password_string
        # Séparer la chaîne hexadécimale en blocs de 2 caractères et convertir chaque bloc en entier hexadécimal
        password_octets = [int(password_string[i:i+2], 16) for i in range(0, len(password_string), 2)]
        cmd_values =bytes([octet2_cmd, octet1_cmd,password_octets[0],password_octets[1],password_octets[2],password_octets[3],password_octets[4],password_octets[5]])
        print (cmd_values)
        response = self.send_command(cmd_values)
        #affichage
        password_string = hex(password)[2:]  # [2:] pour supprimer le préfixe '0x'
        password = bytes.fromhex(password_string)
        password_text = password.decode()
        if response[7] :
            print('set_bluetooth_password success - password =',password_text)
            return 1
        else :
            print('set_bluetooth_password failure ')
            return 0
    
    #2.2.16
    def distance_resolution_setting(self,distance=0x0000) : #0x0000 = 0.75    0x0001 = 0.2   
        cmd = 0x00AA
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        octet1_distance = (distance & 0xFF00) >> 8
        octet2_distance= distance & 0x00FF
        # Créer la séquence d'octets en ordre spécifié
        cmd_distance = bytes([octet2_cmd, octet1_cmd, octet2_distance, octet1_distance])
        response = self.send_command(cmd_distance)
        if response[7] and distance ==0x0000 and not response[8]  :
            print('Distance_resolution_setting 0.75m success')
        elif response[7] and response != NULLDATA and distance ==0x0001 and not response[8]:
            print('Distance_resolution_setting 0.2m success')
            return 1
        else :
            print('Distance_resolution_setting failure ')
            return 0
    
    #2.2.17
    def query_distance_resolution_setting(self) :    
        cmd = 0x00AB
        value = None
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        # Créer la séquence d'octets en ordre spécifié
        cmd_value = bytes([octet2_cmd, octet1_cmd])
        response = self.send_command(cmd_value)
        if response[7] and response[10] and response[4] == 6 :
            print('Distance_resolution_setting 0.2m ')
            return 1
        elif response[7] and response != NULLDATA and  not response[10] and response[4] == 6 :
            print('Distance_resolution_setting 0.75m')
            return 2
        else :
            print('query_distance_resolution_setting failure ')
            return 0
            
    #2.3 RADAR data output
    #2.3.1  envoi d'une commande pour recevoir un rapport             
    def send_command_report_data(self) :
        cmd = 0x0000
        value = None
        #Extraire les octets individuels de chaque valeur -->pour faire little endian 
        octet1_cmd = (cmd & 0xFF00) >> 8
        octet2_cmd = cmd & 0x00FF
        cmd_value = bytes([octet2_cmd, octet1_cmd])
        cmd_data_len = bytes([len(cmd_value), 0x00]) #little endian
        frame = REPORT_HEADER + cmd_data_len + cmd_value + REPORT_TERMINATOR
        self.ser.write(frame)
        #print(frame)#debug
        self.serial_flush() 
        utime.sleep_ms(100)  #tempo assez longue pour récupérer les datas
        #----------------reception  message----------------
        if self.ser.any() > 0: 
            #Lire le message reçu
            report_data = self.ser.read()
            #self.print_trames_bytes(report_data) # debug
            self.parse_report(report_data) #analyse mesure
            return report_data
        else :
            print("probleme communication : reponse vide ")
            report_data = NULLDATA
            # sanity checks passed. Store the sensor data in meas
            self.meas["state"] = 0
            self.meas["moving_distance"] = 0
            self.meas["moving_energy"] = 0
            self.meas["stationary_distance"] = 0
            self.meas["stationary_energy"] = 0
            self.meas["detection_distance"] = 0
            self.communication_error = 1 
            return report_data
        
    def parse_report(self,data):
        # sanity checks
        if len(data) < 23:
            print(f"error, frame length {data} is too short")
            return 0
        if data[0:4] != REPORT_HEADER:
            print(f"error, frame header is incorrect")
            return 0
        # Check if data[4] (frame length) is valid. It must be 0x0d or 0x23
        # depending on if we are in basic mode or engineering mode
        if data[4] != 0x0d and data[4] != 0x23:
            print(f"error, frame length is incorrect")
            return 0
        # data[7] must be report 'head' value 0xaa
        if data[7] != 0xaa:
            print(f"error, frame report head value is incorrect")
            return 0
        # sanity checks passed. Store the sensor data in meas
        self.meas["state"] = data[8]
        self.meas["moving_distance"] = data[9] + (data[10] << 8)
        self.meas["moving_energy"] = data[11]
        self.meas["stationary_distance"] = data[12] + (data[13] << 8)
        self.meas["stationary_energy"] = data[14]
        self.meas["detection_distance"] = data[15] + (data[16] << 8)
        return 1
          
    def print_meas(self):
        print(f"state: {TARGET_NAME[self.meas['state']]}")
        print(f"moving distance: {self.meas['moving_distance']}")
        print(f"moving energy: {self.meas['moving_energy']}")
        print(f"stationary distance: {self.meas['stationary_distance']}")
        print(f"stationary energy: {self.meas['stationary_energy']}")
        print(f"detection distance: {self.meas['detection_distance']}")
        

    def human_detection(self,led,seuil_stat,seuil_mov):
        if self.communication_error :
            for i in range (10) :
                led.on()
                utime.sleep(0.1)
                led.off()
                utime.sleep(0.1)
        elif self.meas['stationary_energy']>seuil_stat or self.meas['moving_energy']>seuil_mov :
            if self.meas['stationary_distance']<self.meas['moving_distance'] :
                print('presence humaine immobile  à ',self.meas['stationary_distance'],'cm')
            else :
                print('presence humaine en mouvement à ',self.meas['moving_distance'],'cm')
            led.on()
            return 1
        else :     
            print('pas de présence humaine')
            led.off()
            return 0