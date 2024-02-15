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

## Estrattore di immagini e imu: da ROS2 a file PNG e CSV
```console
source install/local_setup.bash
ros2 run file_quaternion_publisher from_ros2_images_imu_to_png_csv
```

Si sottoscrive a <b>/camera/left_image</b> e estrae le immagini in una cartella.

<b>NB</b>: Il timestamp che viene assegnato alle immagini e' quello "corrente" (cioe' il timestamp preso al momento dell'estrazione), NON quello contenuto nell'header dei messaggi ros. <br>

Si sottoscrive a <b>/imu/data</b> e ne estrae i dati in un file csv del tipo
```console
# Timestamp,Angular Velocity X,Angular Velocity Y,Angular Velocity Z,Linear Acceleration X,Linear Acceleration Y,Linear Acceleration Z
```
<b>NB</b>: Il timestamp di questo file e', come prima, quello "corrente". Per fare il match con il timestamp originale contenuto nell'header dei messaggi, viene creato il file <i>imu_matching_original_timestamp.csv</i> . In questo file la prima colonna e' il timestamp originale, mentre nella seconda c'e' quello "corrente". 

