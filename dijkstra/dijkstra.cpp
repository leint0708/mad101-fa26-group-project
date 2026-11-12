/**
 * ============================================================================
 * ĐẠI HỌC FPT HÀ NỘI · MAD101 (TOÁN RỜI RẠC)
 * DỰ ÁN: HỆ THỐNG ĐIỀU HƯỚNG DRONE GIAO HÀNG TRÊN KHÔNG
 * ============================================================================
 * Module: Thuật toán Dijkstra tìm đường đi ngắn nhất (Single-Source Shortest Path)
 * Tệp tin: sourcecpp/dijkstra/dijkstra.cpp
 * Tác giả: Nhóm Dự Án MAD101
 * 
 * PHÂN TÍCH ĐỘ PHỨC TẠP:
 * - Thời gian: O((|V| + |E|) log |V|) khi sử dụng Min-Heap (std::priority_queue)
 *   + Khởi tạo khoảng cách: O(|V|)
 *   + Mỗi đỉnh được trích xuất khỏi heap đúng 1 lần: O(|V| log |V|)
 *   + Mỗi cạnh được duyệt tối đa 1 lần trong bước Relax: O(|E| log |V|)
 * - Không gian: O(|V| + |E|) để lưu trữ danh sách kề và mảng khoảng cách/truy vết.
 * 
 * LÝ DO LỰA CHỌN:
 * - Đồ thị giao vận Drone có trọng số khoảng cách và điện năng luôn dương (w(e) > 0).
 * - Thuật toán đảm bảo tìm ra lộ trình tối ưu toàn cục với hiệu năng thời gian thực.
 * ============================================================================
 */

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;

// Hằng số vô cùng đại diện cho khoảng cách chưa chạm tới
const double INF = numeric_limits<double>::infinity();

// Cấu trúc một cạnh có trọng số trong danh sách kề
struct Edge {
    int to;
    double weight;
};

/**
 * Thuật toán Dijkstra tìm đường đi ngắn nhất từ đỉnh nguồn src
 * @param src: Chỉ số đỉnh xuất phát (trạm sạc / trạm kho)
 * @param adj: Danh sách kề biểu diễn đồ thị giao vận vô hướng
 * @param dist: Mảng lưu khoảng cách ngắn nhất từ src đến từng đỉnh
 * @param prev: Mảng lưu đỉnh đi trước để phục vụ truy vết lộ trình
 */
void dijkstra(int src, const vector<vector<Edge>>& adj, 
              vector<double>& dist, vector<int>& prev) {
    int n = adj.size();
    dist.assign(n, INF);
    prev.assign(n, -1);
    
    // Min-heap lưu cặp (khoảng_cách_tích_lũy, chỉ_số_đỉnh)
    using PDI = pair<double, int>;
    priority_queue<PDI, vector<PDI>, greater<PDI>> pq;
    
    // Bước khởi tạo
    dist[src] = 0.0;
    pq.push({0.0, src});
    
    // Vòng lặp chính duyệt đỉnh có nhãn tạm thời nhỏ nhất
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        // Bỏ qua nếu đã tìm thấy đường đi ngắn hơn trước đó
        if (d > dist[u]) continue;
        
        // Bước giãn cạnh (Relaxation) cho tất cả các đỉnh lân cận
        for (const auto& edge : adj[u]) {
            double newDist = dist[u] + edge.weight;
            if (newDist < dist[edge.to]) {
                dist[edge.to] = newDist;
                prev[edge.to] = u;
                pq.push({newDist, edge.to});
            }
        }
    }
}

/**
 * Hàm truy vết lộ trình bay tối ưu từ đỉnh nguồn đến đỉnh đích
 * @param target: Đỉnh đích
 * @param prev: Mảng đỉnh đi trước
 * @return Danh sách thứ tự các đỉnh trên lộ trình bay
 */
vector<int> reconstructPath(int target, const vector<int>& prev) {
    vector<int> path;
    for (int curr = target; curr != -1; curr = prev[curr]) {
        path.push_back(curr);
    }
    reverse(path.begin(), path.end());
    return path;
}

int main(int argc, char* argv[]) {
    // Tối ưu hóa I/O
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    istream* input = &cin;
    ifstream file;
    string datasetName = "stdin";
    
    // Nếu có truyền tên file qua tham số dòng lệnh
    if (argc > 1) {
        file.open(argv[1]);
        if (!file.is_open()) {
            cerr << "[ERROR] Khong the mo file dataset: " << argv[1] << endl;
            cerr << "Cu phap su dung: dijkstra.exe [duong_dan_file_dataset.txt]" << endl;
            return 1;
        }
        input = &file;
        datasetName = argv[1];
    }
    
    string line;
    int V = 0, E = 0, src = 0;
    
    // Đọc số đỉnh V, số cạnh E và đỉnh nguồn src (bỏ qua dòng comment '#')
    while (getline(*input, line)) {
        if (line.empty() || line[0] == '#') continue;
        stringstream ss(line);
        if (ss >> V >> E >> src) break;
    }
    
    if (V == 0) {
        cerr << "[ERROR] Du lieu dataset khong hop le hoac rong!" << endl;
        return 1;
    }
    
    // Danh sách tên trạm định danh theo quy mô Dataset
    vector<string> nodeNames;
    if (V == 6) {
        nodeNames = {"CT2", "Gamma", "CT1", "Dom C", "Dom D", "Dom E"};
    } else {
        nodeNames = {
            "CT1", "CT2", "Alpha", "Beta", "Gamma", "Delta", "Epsilon", "Omega",
            "Zeta", "Dom A", "Dom B", "Dom C", "Dom D", "Dom E", "Dom F", "Dom G"
        };
    }
    
    // Đọc danh sách cạnh và khởi tạo đồ thị vô hướng
    vector<vector<Edge>> adj(V);
    int edgesRead = 0;
    while (edgesRead < E && getline(*input, line)) {
        if (line.empty() || line[0] == '#') continue;
        stringstream ss(line);
        int u, v;
        double w;
        if (ss >> u >> v >> w) {
            adj[u].push_back({v, w});
            adj[v].push_back({u, w}); // Đồ thị vô hướng (Drone bay được cả 2 chiều)
            edgesRead++;
        }
    }
    
    // Thực thi thuật toán Dijkstra
    vector<double> dist;
    vector<int> prev;
    dijkstra(src, adj, dist, prev);
    
    // In kết quả định tuyến có định dạng bảng chuẩn
    cout << "======================================================================" << endl;
    cout << "KET QUA DINH TUYEN DIJKSTRA TU TRAM NGUON: " << nodeNames[src] 
         << " (ID: " << src << ")" << endl;
    cout << "Tap du lieu: " << datasetName << " | So dinh: " << V 
         << " | So canh: " << edgesRead << endl;
    cout << "Do phuc tap: O((|V| + |E|) log |V|) voi std::priority_queue Min-Heap" << endl;
    cout << "======================================================================" << endl;
    cout << left << setw(14) << "Tram dich" 
         << setw(18) << "Khoang cach(m)" 
         << "Tuyen duong bay toi uu" << endl;
    cout << string(70, '-') << endl;
    
    for (int i = 0; i < V; ++i) {
        if (i == src) continue;
        cout << left << setw(14) << nodeNames[i];
        if (dist[i] == INF) {
            cout << setw(18) << "Khong toi duoc" << "Khong co duong bay" << endl;
        } else {
            cout << setw(18) << fixed << setprecision(2) << dist[i];
            vector<int> path = reconstructPath(i, prev);
            for (size_t p = 0; p < path.size(); ++p) {
                cout << nodeNames[path[p]];
                if (p + 1 < path.size()) cout << " -> ";
            }
            cout << endl;
        }
    }
    cout << "======================================================================" << endl;
    
    return 0;
}
