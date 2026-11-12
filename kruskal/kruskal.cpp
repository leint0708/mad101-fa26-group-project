/**
 * ============================================================================
 * ĐẠI HỌC FPT HÀ NỘI · MAD101 (TOÁN RỜI RẠC)
 * DỰ ÁN: HỆ THỐNG ĐIỀU HƯỚNG DRONE GIAO HÀNG TRÊN KHÔNG
 * ============================================================================
 * Module: Thuật toán Kruskal tìm Cây khung nhỏ nhất (Minimum Spanning Tree - MST)
 *         và Cấu trúc Dữ liệu các Tập rời nhau (Disjoint Set Union - DSU)
 * Tệp tin: sourcecpp/kruskal/kruskal.cpp
 * Tác giả: Nhóm Dự Án MAD101
 * 
 * NỘI DUNG MỤC 4.3 & 4.4 BÁO CÁO:
 * - Algorithm 4.5: KruskalMST(V, E, w) — Thuật toán Kruskal xây dựng Cây khung nhỏ nhất
 * - DSU với Nén đường đi (Path Compression) & Gộp theo hạng (Union by Rank)
 * 
 * PHÂN TÍCH ĐỘ PHỨC TẠP:
 * - Thời gian: O(|E| log |V|)
 *   + Sắp xếp |E| cạnh theo trọng số tăng dần: O(|E| log |E|) = O(|E| log |V|) (vì |E| <= |V|^2).
 *   + Khởi tạo DSU (makeSet cho |V| đỉnh): O(|V|).
 *   + Duyệt qua các cạnh và thực hiện 2*|E| thao tác find() + (|V| - 1) thao tác unite():
 *     Nhờ kết hợp Path Compression và Union by Rank, mỗi thao tác DSU chỉ tốn thời gian gần như
 *     hằng số O(alpha(|V|)), trong đó alpha là hàm Ackermann nghịch đảo (alpha(|V|) <= 4 với mọi kích thước thực tế).
 *   + Tổng thời gian: O(|E| log |V| + |E| * alpha(|V|)) = O(|E| log |V|).
 * - Không gian: O(|V| + |E|) để lưu trữ danh sách cạnh và mảng parent/rank của DSU.
 * 
 * LÝ DO LỰA CHỌN:
 * - Đồ thị campus FPT Hòa Lạc là đồ thị thưa (|V| = 16, |E| = 35 << |V|^2 = 256).
 * - Kruskal thao tác trực tiếp trên danh sách cạnh sắp xếp, hiệu quả vượt trội so với Prim trên đồ thị thưa.
 * - Cho phép dễ dàng xác định các cạnh huyết mạch (bottleneck edges) phục vụ kiểm thử sự cố biến đổi mạng (Dataset 3).
 * ============================================================================
 */

#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <set>

using namespace std;

/**
 * Cấu trúc đại diện cho một cạnh có trọng số trong mạng lưới giao vận Drone
 */
struct CampusEdge {
    int u;              // Chỉ số đỉnh đầu
    int v;              // Chỉ số đỉnh cuối
    double weight;      // Khoảng cách / Trọng số tiêu hao năng lượng (mét)
    int originalIndex;  // Thứ tự ban đầu trong dataset
    
    // Nạp chồng toán tử < để sắp xếp cạnh tăng dần theo trọng số
    bool operator<(const CampusEdge& other) const {
        if (abs(weight - other.weight) > 1e-9) {
            return weight < other.weight;
        }
        if (u != other.u) return u < other.u;
        return v < other.v;
    }
};

/**
 * Cấu trúc dữ liệu các tập rời nhau (Disjoint Set Union - DSU)
 * Tích hợp 2 kỹ thuật tối ưu hóa kinh điển:
 * 1. Path Compression (Nén đường đi): Đặt nút cha của mọi nút trên đường duyệt trỏ thẳng về gốc.
 * 2. Union by Rank (Gộp theo hạng): Gắn cây có độ cao nhỏ hơn vào gốc của cây có độ cao lớn hơn.
 */
struct DisjointSetUnion {
    vector<int> parent;
    vector<int> rank;
    int numSets;
    
    DisjointSetUnion(int n) : parent(n), rank(n, 0), numSets(n) {
        iota(parent.begin(), parent.end(), 0); // Ban đầu mỗi đỉnh là 1 tập độc lập: parent[i] = i
    }
    
    /**
     * Thao tác FIND: Tìm đại diện (gốc) của tập hợp chứa phần tử i
     * Kỹ thuật Nén đường đi (Path Compression)
     */
    int find(int i) {
        if (parent[i] == i) return i;
        return parent[i] = find(parent[i]); // Đệ quy và gán lại cha trực tiếp
    }
    
    /**
     * Thao tác UNITE: Hợp nhất hai tập hợp chứa phần tử i và j
     * Kỹ thuật Gộp theo hạng (Union by Rank)
     * @return true nếu hợp nhất thành công; false nếu i và j đã thuộc cùng một tập (tạo chu trình!)
     */
    bool unite(int i, int j) {
        int rootI = find(i);
        int rootJ = find(j);
        
        // Nếu cùng gốc -> thêm cạnh (i, j) sẽ TẠO CHU TRÌNH! Bỏ qua!
        if (rootI == rootJ) return false;
        
        // Gộp theo hạng để giữ chiều cao cây luôn ở mức tối thiểu O(log |V|)
        if (rank[rootI] < rank[rootJ]) {
            parent[rootI] = rootJ;
        } else if (rank[rootI] > rank[rootJ]) {
            parent[rootJ] = rootI;
        } else {
            parent[rootJ] = rootI;
            rank[rootI]++;
        }
        numSets--;
        return true;
    }

    /**
     * Lấy danh sách các tập hợp liên thông hiện tại (phục vụ in trực quan trạng thái DSU)
     */
    map<int, vector<int>> getDisjointComponents(int n) {
        map<int, vector<int>> components;
        for (int i = 0; i < n; ++i) {
            components[find(i)].push_back(i);
        }
        return components;
    }
};

/**
 * ALGORITHM 4.5: KruskalMST(V, E, w)
 * Thuật toán Kruskal xây dựng Cây khung nhỏ nhất
 * @param numVertices: Số lượng đỉnh |V|
 * @param edges: Danh sách tất cả các cạnh E
 * @param mstEdges: Vector lưu các cạnh được chọn vào cây khung E_T
 * @param stepLogs: Vector ghi lại nhật ký từng bước kiểm tra cạnh
 * @return Tổng trọng số của cây khung nhỏ nhất (Total MST Weight)
 */
double kruskalMST(int numVertices, vector<CampusEdge>& edges, 
                  vector<CampusEdge>& mstEdges, 
                  vector<string>& stepLogs,
                  const vector<string>& nodeNames) {
    // Bước 1: Sắp xếp tất cả các cạnh theo thứ tự trọng số không giảm
    sort(edges.begin(), edges.end());
    
    // Bước 2: Khởi tạo DSU với |V| tập rời nhau
    DisjointSetUnion dsu(numVertices);
    mstEdges.clear();
    stepLogs.clear();
    double totalWeight = 0.0;
    
    // Bước 3: Duyệt lần lượt từng cạnh trong danh sách đã sắp xếp
    int step = 0;
    for (const auto& edge : edges) {
        step++;
        stringstream ss;
        string uName = (edge.u < (int)nodeNames.size()) ? nodeNames[edge.u] : to_string(edge.u);
        string vName = (edge.v < (int)nodeNames.size()) ? nodeNames[edge.v] : to_string(edge.v);
        
        int rootU = dsu.find(edge.u);
        int rootV = dsu.find(edge.v);
        
        if (rootU != rootV) {
            // Không tạo chu trình: CHỌN CẠNH VÀO CÂY KHUNG
            dsu.unite(edge.u, edge.v);
            mstEdges.push_back(edge);
            totalWeight += edge.weight;
            
            ss << "[Buoc " << setw(2) << step << "] CHON   : " 
               << left << setw(10) << (uName + " - " + vName) 
               << " | W = " << fixed << setprecision(2) << setw(7) << edge.weight << "m"
               << " | Gop goc (" << rootU << ", " << rootV << ") -> |E_T| = " << mstEdges.size() << "/" << (numVertices - 1);
            stepLogs.push_back(ss.str());
            
            // Tính chất MST: Cây khung luôn có đúng |V| - 1 cạnh -> DỪNG SỚM (Early Break)
            if (mstEdges.size() == static_cast<size_t>(numVertices - 1)) {
                stepLogs.push_back("-> [DUNG SOM] Da chon du |V| - 1 = " + to_string(numVertices - 1) + " canh! Cay khung da lien thong toan bo.");
                break;
            }
        } else {
            // Đã cùng gốc: BỎ QUA DO TẠO CHU TRÌNH
            ss << "[Buoc " << setw(2) << step << "] BO QUA : " 
               << left << setw(10) << (uName + " - " + vName) 
               << " | W = " << fixed << setprecision(2) << setw(7) << edge.weight << "m"
               << " | [CHU TRINH] Cung thuoc tap goc ID: " << rootU;
            stepLogs.push_back(ss.str());
        }
    }
    
    return totalWeight;
}

int main(int argc, char* argv[]) {
    // Tối ưu hóa I/O
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    istream* input = &cin;
    ifstream file;
    string datasetPath = "dataset1.txt";
    
    if (argc > 1) {
        datasetPath = argv[1];
        file.open(datasetPath);
        if (!file.is_open()) {
            cerr << "[ERROR] Khong the mo file dataset: " << datasetPath << endl;
            cerr << "Cu phap su dung: kruskal.exe [duong_dan_file_dataset.txt]" << endl;
            return 1;
        }
        input = &file;
    } else {
        // Mặc định thử mở dataset1.txt nếu không truyền tham số
        file.open(datasetPath);
        if (file.is_open()) {
            input = &file;
        }
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
    
    // Danh sách tên trạm định danh
    vector<string> nodeNames;
    if (V == 6) {
        nodeNames = {"CT2", "Gamma", "CT1", "Dom C", "Dom D", "Dom E"};
    } else {
        nodeNames = {
            "CT1", "CT2", "Alpha", "Beta", "Gamma", "Delta", "Epsilon", "Omega",
            "Zeta", "Dom A", "Dom B", "Dom C", "Dom D", "Dom E", "Dom F", "Dom G"
        };
    }
    
    // Đọc danh sách tất cả các cạnh
    vector<CampusEdge> edges;
    int edgeCount = 0;
    while (edgeCount < E && getline(*input, line)) {
        if (line.empty() || line[0] == '#') continue;
        stringstream ss(line);
        int u, v;
        double w;
        if (ss >> u >> v >> w) {
            edges.push_back({u, v, w, edgeCount++});
        }
    }
    
    cout << "======================================================================" << endl;
    cout << "THUAT TOAN KRUSKAL - XAY DUNG CAY KHUNG NHO NHAT (MST) VOI DSU" << endl;
    cout << "MAD101 - Toan Roi Rac | Mon hoc: Discrete Mathematics (FPT University)" << endl;
    cout << "======================================================================" << endl;
    cout << "Tap du lieu: " << datasetPath << endl;
    cout << "So luong dinh (|V|): " << V << " | So luong canh (|E|): " << edges.size() << endl;
    cout << "Do phuc tap ly thuyet: O(|E| log |V|) + O(|E| * alpha(|V|))" << endl;
    cout << "======================================================================" << endl << endl;

    // 1. Thực thi thuật toán Kruskal
    vector<CampusEdge> mstEdges;
    vector<string> stepLogs;
    double totalMSTWeight = kruskalMST(V, edges, mstEdges, stepLogs, nodeNames);

    // 2. In nhật ký từng bước kiểm tra cạnh
    cout << "NHAT KY TUNG BUOC THUC THI KRUSKAL & KIEM TRA CHU TRINH DSU:" << endl;
    cout << string(70, '-') << endl;
    for (const auto& log : stepLogs) {
        cout << log << endl;
    }
    cout << string(70, '-') << endl << endl;

    // 3. In bảng kết quả các cạnh được chọn vào cây khung MST
    cout << "DANH SACH CAC CANH THUOC CAY KHUNG NHO NHAT (MST):" << endl;
    cout << string(70, '-') << endl;
    cout << left << setw(6)  << "STT" 
         << setw(16) << "Dinh u" 
         << setw(16) << "Dinh v" 
         << setw(18) << "Chieu dai cap(m)" 
         << "Y nghia ha tang" << endl;
    cout << string(70, '-') << endl;

    for (size_t i = 0; i < mstEdges.size(); ++i) {
        const auto& e = mstEdges[i];
        string uName = (e.u < (int)nodeNames.size()) ? nodeNames[e.u] : to_string(e.u);
        string vName = (e.v < (int)nodeNames.size()) ? nodeNames[e.v] : to_string(e.v);
        
        cout << left << setw(6) << (i + 1)
             << setw(16) << uName
             << setw(16) << vName
             << setw(18) << fixed << setprecision(2) << e.weight;
             
        if ((uName == "CT1" && vName == "Dom C") || (uName == "Dom C" && vName == "Dom D")) {
            cout << "[CANH HUYET MACH]";
        } else {
            cout << "Lien ket khu vuc";
        }
        cout << endl;
    }
    cout << string(70, '-') << endl;
    cout << fixed << setprecision(2);
    cout << "-> TONG SO CANH TRONG MST: " << mstEdges.size() << " / " << (V - 1) << " canh" << endl;
    cout << "-> TONG CHIEU DAI CAP LIEN THONG TOAN BO " << V << " KHU VUC: " << totalMSTWeight << " m" << endl;
    cout << "======================================================================" << endl << endl;

    // 4. Đánh giá ý nghĩa thực tiễn & kiểm thử khả năng chịu lỗi
    cout << "DANH GIA Y NGHIA THUC TIEN & CHI PHI HA TANG:" << endl;
    cout << "1. Toi uu chi phi: Cay khung Kruskal giup ket noi toan bo " << V 
         << " khu vuc ma khong tao chu trinh du thua, giam toi da chi phi keo cap tin hieu & tram sac." << endl;
    cout << "2. Kiem thu su co (Dataset 3): Neu canh huyet mach bi dut gay, he thong DSU co the tai cau truc" << endl;
    cout << "   lai cay khung voi cac tuyen du phong thay the ma van dam bao 100% lien thong." << endl;
    cout << "======================================================================" << endl;

    return 0;
}
