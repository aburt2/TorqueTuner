# Torquetuner Connection Guide V1.0

- Torquetuner Connection Guide
    - Option 1: Puara Serial Manager
        - Set up Puara Serial Manager
        - Configurer config.json file
        - Upload to Torquetuner Module
    - Option 2: Wireless Update
## Option 1: Puara Serial Manager
1. Download or clone the T-Tree repository at https://github.com/IDMIL/T-Tree.
2. In the client folder open `config_template.json`.
3. Put in the network name you want the Torquetuner to connect to in the __wifiSSID__ field.
4. Put the network password in the __wifiPSK__ field.
5. Save and close `config_template.json`.
6. Run the `puara_serial_manager.py` script.
7. Connect the Torquetuner module to your PC using a USB cable.
8. The script should auto detect and configure the Torquetuner module.

## Option 2: Wireless Connection
### Get your network details
1. Connect to the network you will be connecting the Torquetuner module to.
2. Note the SSID (network name) and SSID Password (network password).
3. Get your computers IP address while connected to this network.

### Connect to the Torquetuner Module.
4. Power on your Torquetuner module and wait until the boot sequence is complete.
5. Connect to the TorqueTuner_XXX wifi where XXX is the ID of the Torquetuner module. ie: TorqueTuner_001. By default the password is mappings.
6. A page should automatically open showing the Torquetuner configuration menu. If not open your browser and go to http://192.168.4.1.
7. In the __Network__ section write the network name and password in the __SSID__ and __SSID Password__ fields. 
8. In the __OSC send settings__ put in your computer's IP address optained in Step 3 in the __Primary IP__ field.
8. Click the green __Save__ button.
6. Click __Config__ on the top of the page to return to the orginal menu.
7. Then Click the __Close and Reboot__ button at the bottom of the page.