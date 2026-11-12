# MÃ NGUỒN THUẬT TOÁN VIẾT BẰNG C++

**Dự Án: Hệ Thống Điều Hướng Drone Giao Hàng Trên Không**  
**Môn học: MAD101 - Toán Rời Rạc (Discrete Mathematics) · Đại học FPT Hà Nội**

---

## 1. Cấu trúc thư mục (Directory Structure)

Thư mục `mad101-fa26-group-project/` được tổ chức độc lập cho từng thuật toán cốt lõi của dự án:

```
mad101-fa26-group-project/
├── dijkstra/
│   ├── dataset/
│   │   ├── dataset1.txt  # Dataset 1
│   │   ├── dataset2.txt  # Dataset 2
│   │   └── dataset3.txt  # Dataset 3
│   └── dijkstra.cpp      # Thuật toán Dijkstra tìm đường đi ngắn nhất (Single-Source Shortest Path)
├── huffman/
│   ├── huffman.cpp       # Thuật toán xây dựng cây Huffman, duyệt tiền thứ Preorder & giải mã luồng bit
│   └── huffman.exe       # File thực thi đã biên dịch sẵn
├── kruskal/
├── dataset/
│   │   ├── dataset1.txt  # Dataset 1
│   │   ├── dataset2.txt  # Dataset 2
│   │   └── dataset3.txt  # Dataset 3
│   ├── kruskal.cpp       # Thuật toán Kruskal tìm Cây khung nhỏ nhất (MST) & Cấu trúc DSU
│   └── mst.cpp           # Bản sao đồng bộ tương thích
└── README.md             # Hướng dẫn chi tiết sử dụng & kiểm thử
```

---

## 2. Hướng dẫn biên dịch & chạy thử nghiệm (How to Compile & Run)

Yêu cầu môi trường: Trình biên dịch C++ hỗ trợ C++17 trở lên (`g++`, `clang++`, hoặc `MSVC`).

### 2.1. Thuật toán Dijkstra (`mad101-fa26-group-project/dijkstra/`)

- **Biên dịch:**
  ```cmd
  cd mad101-fa26-group-project/dijkstra
  g++ -std=c++17 -O2 dijkstra.cpp -o dijkstra.exe
  ```
- **Chạy trên Dataset 1 (6 đỉnh, 9 cạnh):**
  ```cmd
  dijkstra.exe dataset/dataset1.txt
  ```
- **Chạy trên Dataset 2 (16 đỉnh, 35 cạnh):**
  ```cmd
  dijkstra.exe dataset/dataset2.txt
  ```
- **Chạy trên Dataset 3 (16 đỉnh, 33 cạnh - Sự cố cắt đường):**
  ```cmd
  dijkstra.exe dataset/dataset3.txt
  ```

### 2.2. Thuật toán Huffman (`mad101-fa26-group-project/huffman/`)

- **Biên dịch:**
  ```cmd
  cd mad101-fa26-group-project/huffman
  g++ -std=c++17 -O2 huffman.cpp -o huffman.exe
  ```
- **Chạy kiểm thử mặc định (Tập lệnh Drone: FORWARD, LEFT, RIGHT, HOVER, RECHARGE):**
  ```cmd
  huffman.exe
  ```
- **Chạy giải mã luồng bit tùy chỉnh (ví dụ bitstream "0101101111"):**
  ```cmd
  huffman.exe 0101101111
  ```

### 2.3. Thuật toán Kruskal (`mad101-fa26-group-projecte/kruskal/`)

- **Biên dịch:**
  ```cmd
  cd mad101-fa26-group-project/kruskal
  g++ -std=c++17 -O2 kruskal.cpp -o kruskal.exe
  ```
- **Chạy trên Dataset 1 (Kiểm thử chu trình & dừng sớm |E_T| = 5):**
  ```cmd
  kruskal.exe dataset/dataset1.txt
  ```
- **Chạy trên Dataset 2 (Cây khung xương sống 16 trạm campus):**
  ```cmd
  kruskal.exe dataset/dataset2.txt
  ```
- **Chạy trên Dataset 3 (Kiểm thử tái cấu trúc mạng khi đứt gãy 2 cạnh huyết mạch):**
  ```cmd
  kruskal.exe dataset/dataset3.txt
  ```

---

## 3. Bảng tổng hợp thuật toán & độ phức tạp

| Thuật toán   | Vị trí mục báo cáo             | Cấu trúc dữ liệu chính                                        | Độ phức tạp thời gian                          | Độ phức tạp không gian | Ý nghĩa thực tiễn                                                                                            |
| :----------- | :----------------------------- | :------------------------------------------------------------ | :--------------------------------------------- | :--------------------- | :----------------------------------------------------------------------------------------------------------- |
| **Dijkstra** | Mục 4.1 & 4.4 (`dijkstra.cpp`) | Min-Heap (`std::priority_queue`), Danh sách kề                | $O((\|V\| + \|E\|) \log \|V\|)$                | $O(\|V\| + \|E\|)$     | Định tuyến đường bay tiêu hao ít năng lượng pin nhất từ trạm Căng tin CT2                                    |
| **Huffman**  | Mục 4.2 & 4.4 (`huffman.cpp`)  | Cây nhị phân đầy đủ (`HuffmanNode`), Min-Heap                 | $O(n \log n)$ tạo cây, $O(m)$ giải mã          | $O(n)$                 | Nén tín hiệu điều khiển không gian vô tuyến, tiết kiệm **30.0% băng thông** ($2.10$ vs $3.00$ bits)          |
| **Kruskal**  | Mục 4.3 & 4.4 (`kruskal.cpp`)  | Disjoint Set Union (DSU) với Path Compression & Union by Rank | $O(\|E\| \log \|V\|) + O(\|E\| \alpha(\|V\|))$ | $O(\|V\| + \|E\|)$     | Tối ưu hóa tổng chiều dài cáp liên thông toàn campus ($465.79\text{ m}$ trên D1) và xác định cạnh huyết mạch |
