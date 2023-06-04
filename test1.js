import BleManager from 'react-native-ble-plx';

const bleManager = new BleManager();

bleManager.start({ showAlert: false })
  .then(() => {
    console.log('Bluetooth initialized');
  })
  .catch(error => {
    console.error('Error initializing Bluetooth:', error);
  });

  bleManager.startDeviceScan(null, null, (error, device) => {
    if (error) {
      console.error('Error scanning for devices:', error);
      return;
    }
  
    // Filter and connect to the desired device
    if (device.name === 'Navi') {
      bleManager.stopDeviceScan();
  
      device.connect()
        .then(device => {
          console.log('Connected to BLE device:', device.name);
          // Perform further operations on the device
        })
        .catch(error => {
          console.error('Error connecting to BLE device:', error);
        });
    }
  });

device.discoverAllServicesAndCharacteristics()
  .then(device => {
    return device.characteristicsForService('a1f6138b-8008-4714-9846-f10a06c95bcb');
  })
  .then(characteristics => {
    const characteristic = characteristics.find(c => c.uuid === '174c9b66-ee3d-4378-a303-eb507131280a');

    if (characteristic) {
      characteristic.monitor((error, characteristic) => {
        if (error) {
          console.error('Error enabling notifications:', error);
          return;
        }

        const value = characteristic.value;
        // Process the received value
      });
    }
  })
  .catch(error => {
    console.error('Error discovering services and characteristics:', error);
  });

device.cancelConnection()
  .then(() => {
    console.log('Disconnected from BLE device');
  })
  .catch(error => {
    console.error('Error disconnecting from BLE device:', error);
  });
