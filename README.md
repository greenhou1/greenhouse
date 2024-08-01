                                  ![Logo GreenHouse Negro](https://github.com/user-attachments/assets/63d07c7c-8838-4c60-a02c-31fb523c3f48)

Descripción del proyecto
GREENHOUSE se enfoca en la implementación de un sistema de gestión de riego agrícola por goteo, tiene como objetivo optimizar el uso del agua en mini-invernaderos. Mediante el monitoreo y control automatizado del riego, este sistema busca maximizar la eficiencia del recurso hídrico y potenciar la productividad agrícola.
El sistema se compone de sensores de humedad del suelo y temperatura ambiental, los cuales recopilan datos en tiempo real sobre las condiciones del mini-invernadero. Estos datos son procesados por un microcontrolador ESP32 y una Raspberry Pi, que actúa como servidor. La comunicación entre los sensores y el sistema de control se facilita a través de un servidor MQTT.
Para la gestión remota, se ha desarrollado un dashboard local utilizando NodeRED, alojado en la Raspberry Pi conectada a una red LAN. Este dashboard permite a los usuarios monitorear y controlar el sistema de riego desde cualquier lugar. Adicionalmente, se integra una base de datos en tiempo real utilizando Firebase, garantizando la sincronización inmediata de los datos y una respuesta oportuna a las necesidades de riego.
El sistema también ofrece control remoto desde cualquier ubicación gracias a la implementación de un broker MQTT alojado en un servidor en la nube de Amazon Web Services. Esto permite el monitoreo y control del sistema desde cualquier dispositivo y lugar. Para una experiencia de control más intuitiva y cómoda, se ha utilizado una aplicación de software libre (Iot MQTT Panel) para dispositivos móviles Android, la cual permite controlar y visualizar el sistema tanto local como remotamente.

![Captura de pantalla 2024-07-30 215658](https://github.com/user-attachments/assets/ffe70244-1045-4e41-a977-edd59f9abc11)
![Captura de pantalla 2024-07-30 210005](https://github.com/user-attachments/assets/e6b99c1c-0dd5-48f2-964d-c669003ecc42)
![Captura de pantalla 2024-07-30 215711](https://github.com/user-attachments/assets/81fb28e2-d07d-4b9b-920a-f357e08b2633)
![Captura de pantalla 2024-07-30 215643](https://github.com/user-attachments/assets/d3a1bc2e-7037-40b8-b65b-a43bef736d37)
