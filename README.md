# From file to quaternion publisher

## Build
```console
colcon build
```

## Da file txt a pubblicazione di quaternioni per RVIZ2
```console
source install/local_setup.bash
ros2 run file_quaternion_publisher file_quaternion_publisher /path/to/file_with_quaternions.txt [tempo_sleep]
```
Parametri:
- Il file da fornire e' quello restituito da orbslam alla fine dell'esecuzione (f_nome.txt). Deve contenere N righe con N=numero immagini processate. Ogni riga contiene le seguenti 8 colonne separate da spazi:un valore che viene scartato,
 position x, position y, position z, quaternion x, quaternion y, quaternion z, quaternion w.
 
- tempo_sleep e' la distanza (in millisecondi) tra una pubblicazione di un'odometry e la successiva. Default: 100.

Il programma pubblica sui topics:
- odom/original (odometry originale, contenuta nel file txt) 
- odom/prova (nuova odometry calcolata/ruotata )

## Check odometry uguale
```
ros2 run file_quaternion_publisher check_odometry_uguale /path/to/file_with_quaternions_effettivi.txt  /path/to/file_with_quaternions_orbslam.txt [tempo_sleep]
```
Primo file: quello che contiene l'odometry giusta. Secondo file: quello di orbslam. Entrambi sono nello stesso formato del file_quaternion_publisher.cpp.
Su RVIZ2: le frecce verdi sono quelle efettive, quelle rosse sono quelle di orbslam
tempo_sleep e' la distanza (in millisecondi) tra una pubblicazione di un'odometry e la successiva. Default: 100.



## TF messages
```
ros2 run file_quaternion_publisher publish_tf_orbslam /path/to/file_with_quaternions.txt [tempo_sleep]
```

Questo script pubblica l'odometry (sul topic <i>/odom_orbslam</i>) calcolata da orbslam ed i relativi tf messages (su <i>/tf</i>). Legge il file con le odometry gia' computate, ma serve per simulare la Track di orbslam.

Il default del tempo_sleep e' 70 millisecondi (quindi rate di 14 hz come le fps della zed).


Per la bag del 7 marzo con la zed2 -> Consiglio: invece di runnare il file-quaternion-publisher da linea di comando, usa  lo script <b>esegui_tf.sh</b> che fa partire in contemporanea anche fast lio e la bag. <B> Nello script bash cambia i path. </B> FIXME ->  Lo script non viene fermato. Bisogna aspettare che termini da solo la pubblicazione di tutti i tf :') 

Su un altro terminale fai  
```
ros2 run file_quaternion_publisher tf_listener
```
Questo script legge periodicamente su <i>/tf</i> e calcola la trasformazione dell'odometry di orbslam rispetto a quella di fastlio. (Idealmente dovrebbe pubblicare poi la nuova odometry su <i>/odom_tf_translated</i> ma la pubblicazione non e' corretta). Ignora le scritte iniziali che vengono stampate. 


Su un altro terminale fai  
```
rviz2 -d tf_transform.rviz
```
Si vedra': 
- in verde: la traiettoria di fastlio
- in rosso: quella di orbslam (quella originale, non ruotata)
- i tf messages

FIXME: forse la pubblicazione di orbslam e quella di faslio non sono del tutto sincronizzate. La lookupTransform non si lamenta ma forse perche' non tiene conto del timestamp. 


## Estrattore di immagini e imu: da ROS2 a file PNG e CSV
```console
source install/local_setup.bash
ros2 run file_quaternion_publisher from_ros2_images_imu_to_png_csv
```

Si sottoscrive alla camera sinistra e alla destra e estrae le immagini nelle cartelle "left" e  "right".

Si sottoscrive a <b>/imu/data</b> e ne estrae i dati in un file csv del tipo
```console
# Timestamp,Angular Velocity X,Angular Velocity Y,Angular Velocity Z,Linear Acceleration X,Linear Acceleration Y,Linear Acceleration Z
```
