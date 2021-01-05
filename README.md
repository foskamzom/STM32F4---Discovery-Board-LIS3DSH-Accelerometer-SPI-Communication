# STM32F4---Discovery-Board-LIS3DSH-Accelerometer-SPI-Communication

In this project, I implemented a basic Android Orientation system, using the STM32F4 Discovery board and the onboard LIS3DSH Accelerometer sensor.
Code is mostly generated by STM32CubeMX, but descriptions can be found over every user defined variables and functions. STM HAL (Hardware Abstraction Layer)
library is used in this example. 

LIS3DSH and Discovery board are connected with pins A5 (Serial Clock), A6 (Master-In / Slave-Out), A7 (Master-Out / Slave-In) and E3 (Chip Select). Pins D12, D13, D14 and D15
(onboard leds with colors green, orange, red and blue respectfully) are used for visualizing the output.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Bu projede STM32F4 Discovery kartını ve kart üzerinde mevcut olan LIS3DSH ivme ölçer sensörünü kullanarak basit bir Android Oryantasyon sistemi oluşturdum.
Kodun büyük çoğunluğu STM32CubeMX tarafından oluşturuldu, fakat kullanıcı tanımlı bütün değişkenler ve fonksiyonların üzerinde bir açıklama kısmı bulunmaktadır.
Bu örnekte STM HAL (Donanım Soyutlaştırma Katmanı) kütüphanesi kullanılmıştır.

LIS3DSH ve Discovery kart A5 (Serial Clock), A6 (Master-In / Slave-Out), A7 (Master-Out / Slave-In) ve E3 (Chip Select) pinleri ile birbirine bağlıdır. 
D12, D13, D14 ve D15 pinleri (Discovery kart üzerinde bulunan sırasıyla yeşil, turuncu, kırmızı ve mavi ledler) sonucun görselleştirilmesi için kullanılmaktadır.

