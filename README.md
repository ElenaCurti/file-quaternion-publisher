# From file to quaternion publisher

## Build
```console
colcon build
```

## Run
```console
source install/local_setup.bash
ros2 run file_quaternion_publisher file_quaternion_publisher /path/to/file_with_quaternions.txt
```

Il file da fornire e' quello restituito da orbslam alla fine dell'esecuzione (f_nome.txt). Deve contenere N righe con N=numero immagini processate. Ogni riga contiene le seguenti 8 colonne separate da spazi:un valore che viene scartato,
 position x, position y, position z, quaternion x, quaternion y, quaternion z, quaternion w.
 
