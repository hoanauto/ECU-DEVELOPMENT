## AUTOMOTIVE ECU DEVELOPMENT 2025
<details><summary>LESSON 1: FIRST PROJECT WITH KEILC</summary>
    <p>
        
## LESSON 1: FIRST PROJECT WITH KEILC
### 1. Keilc
`KeilC` là một môi trường phát triển tích hợp (IDE) phổ biến được sử dụng để lập trình các vi điều khiển, đặc biệt là dòng vi điều khiển ARM Cortex-M, bao gồm STM32, LPC, Kinetis, và nhiều dòng vi điều khiển khác.
#### 1.1 Các thành phần chính của KeilC
- KeilC bao gồm các thành phần quan trọng sau:
  - µVision IDE: Giao diện tích hợp để viết, biên dịch và debug code.
  - C/C++ Compiler (Arm Compiler): Trình biên dịch mạnh mẽ giúp tối ưu hóa mã nguồn.
  - Debugger: Công cụ gỡ lỗi hỗ trợ mô phỏng và debug trực tiếp trên phần cứng.
  - RTX Real-Time Operating System (RTOS): Hệ điều hành thời gian thực cho vi điều khiển.
#### 1.2. Đặc điểm nổi bật của KeilC
- Hỗ trợ vi điều khiển ARM Cortex-M: KeilC là lựa chọn hàng đầu khi lập trình STM32.
- Không cần CubeMX: Người dùng có thể tự viết code mà không phụ thuộc vào các file tự động sinh của CubeMX.
- Hỗ trợ Debug mạnh mẽ: Cho phép debug qua JTAG, SWD, và giả lập.
- Thư viện phong phú: Cung cấp nhiều thư viện hỗ trợ phát triển ứng dụng nhúng.
- Giao diện trực quan: Dễ dàng cấu hình, quản lý dự án và biên dịch code.

![image](https://github.com/user-attachments/assets/343dceb4-8bb7-4e19-9910-e4676cbe6352)
### 2.Tạo project đầu tiên với Keilc
#### 2.1. Cài đặt
- Cài đặt Keil C về máy :https://www.keil.com/download/
- Cài đặt ST-link driver để nạp và debug: https://www.keil.com/download/
- Cài đặt thư viện chuẩn của STM32F103c8t6 cho keil: https://www.keil.arm.com/devices/
#### 2.2. Tạo project
- Ở màn hình chính của KeilC, trong mục Project, chọn New uVision Project, chọn thư mục lưu và đặt tên.
- Sau khi project được tạo, KeilC sẽ hiện cửa sổ để người dùng chọn dòng MCU sử dụng. Ở đây là STM32F103C8
![image](https://github.com/user-attachments/assets/e08a16bb-667c-477a-abb1-d470ee5c1cf7)
Nhấn OK, chuyển sang cửa sổ để thêm các files và thư viện cần thiết.
![image](https://github.com/user-attachments/assets/9f98fd97-8db1-4205-b58e-a181a07deb5e)
Tạo file main.c và add vào project.
![image](https://github.com/user-attachments/assets/18823c59-1d35-4f5d-bc40-b3fe784e76d0)
có thể thử build để kiểm tra xem có lỗi khi link thư viện không.
### 3. Blink Led PC13
Kit Bluepill có sẵn 1 user led trên board, led này nối tới chân 13 của GPIOC (chân PC13), nên chúng ta sẽ dùng nó để test hoạt động trong bài đầu tiên.
![image](https://github.com/user-attachments/assets/26fab549-ad5d-4b9e-ab35-c811514cf604)
- Để Blink led PC13 cần thực hiện 3 bước:
  - Cấp xung clock cho ngoại vi
  - Cấu hình chân của ngoại vi
  - Sử dụng ngoại vi
#### 3.1. Cấp xung clock cho ngoại vi
Để cấu hình hoạt động cho các ngoại vi, trước hết cần cấu hình cấp xung clock cho chúng qua các bus.
- Các ngoại vi được cấp xung clock thông qua các đường bus sau:
  - `AHB (Advanced High Speed Buses)`: Đây là Bus kết nối hệ thống.
  - `APB1, APB2 (Advanced Peripheral Buses 1,2)`: Đây là các Bus kết nối với thiết bị ngoại vi và kết nối với hệ thống thông qua AHB.
-> Để cấu hình Clock cho một ngoại vi, chúng ta cần nắm được ngoại vi đó được cấp clock từ đường bus nào (AHB/APB). 
![image](https://github.com/user-attachments/assets/9cc317d1-ca02-4e6d-a194-83899628b0e5)
- Nhìn sơ đồ ta thấy được, **các GPIO được cấp clock từ APB2**
- **RCC (Reset and Clock Control)** là bộ điều khiển xung nhịp trên STM32, chịu trách nhiệm quản lý nguồn xung nhịp cho CPU, GPIO, bộ nhớ và các ngoại vi khác. Trong STM32F103, RCC có địa chỉ cơ bản là `0x40021000`.
- **APB2** được cấu hình bởi **thanh ghi APB2 peripheral clock enable register (RCC_APB2ENR)**. 
![image](https://github.com/user-attachments/assets/d5bb76c8-7c0d-4bf0-aa03-5f37fa297e8e)
- **thanh ghi APB2 peripheral clock enable register (RCC_APB2ENR)** được định nghĩa với `offset` (độ lệch) so với địa chỉ cơ bản là `0x18`
--> địa chỉ đầy đủ của thanh ghi RCC_APB2ENR là `0x40021018` và được định nghĩa như sau
  ```c
  #define RCC_APB2ENR *((unsigned int*)0x40021018)
- Set bit IOPCEN lên 1 để cấp clock cho GPIOC.
  ```c
    RCC_APB2ENR |= (1<<4);
#### 3.2. Cấu hình chế độ hoạt động
