#include <bits/stdc++.h>
using namespace std;

/**
 * Cấu trúc một nút trên Cây nhị phân đầy đủ Huffman
 */
struct HuffmanNode {
    string symbol;                     // Tên lệnh 
    double freq;                       // Tần suất xuất hiện 
    int order;                         // Thứ tự khởi tạo để đảm bảo thứ tự gộp tất định
    shared_ptr<HuffmanNode> left;      // Con trái (tương ứng bit 0)
    shared_ptr<HuffmanNode> right;     // Con phải (tương ứng bit 1)
    
    HuffmanNode(string s, double f, int ord = 0) 
        : symbol(s), freq(f), order(ord), left(nullptr), right(nullptr) {}
        
    bool isLeaf() const {
        return left == nullptr && right == nullptr;
    }
};

struct CompareNode {
    bool operator()(const shared_ptr<HuffmanNode>& a, 
                    const shared_ptr<HuffmanNode>& b) const {
        if (abs(a->freq - b->freq) > 1e-9) {
            return a->freq > b->freq; // Tần suất nhỏ hơn có độ ưu tiên cao hơn
        }
        // Khi tần suất bằng nhau: ưu tiên nút nội bộ đã gộp trước
        if (a->isLeaf() != b->isLeaf()) {
            return a->isLeaf(); // a là lá thì độ ưu tiên thấp hơn (nằm sau trong Min-Heap)
        }
        return a->order > b->order;
    }
};

/**
 * ALGORITHM 4.2: BuildHuffmanTree(C, F)
 * Xây dựng Cây nhị phân Huffman tối ưu bằng chiến lược tham lam (Greedy)
 */
shared_ptr<HuffmanNode> buildHuffmanTree(const vector<pair<string, double>>& freqTable) {
    priority_queue<shared_ptr<HuffmanNode>, 
                   vector<shared_ptr<HuffmanNode>>, 
                   CompareNode> pq;
    
    int counter = 0;
    for (const auto& [sym, freq] : freqTable) {
        pq.push(make_shared<HuffmanNode>(sym, freq, ++counter));
    }
    
    while (pq.size() > 1) {
        auto n1 = pq.top(); pq.pop();
        auto n2 = pq.top(); pq.pop();
        
        shared_ptr<HuffmanNode> leftNode, rightNode;
        if (!n1->isLeaf() && n2->isLeaf()) {
            leftNode = n2;
            rightNode = n1;
        } else {
            leftNode = n1;
            rightNode = n2;
        }
        
        auto parent = make_shared<HuffmanNode>("", leftNode->freq + rightNode->freq, ++counter);
        parent->left = leftNode;
        parent->right = rightNode;
        pq.push(parent);
    }
    
    return pq.top(); // Gốc của cây nhị phân tối ưu
}

/**
 * ALGORITHM 4.3: PreorderGenerateCodes(root, code, codeTable)
 * Duyệt cây tiền thứ (Preorder Traversal) để sinh bảng mã tiền tố
 * Quy tắc: Gặp nút lá -> ghi nhận mã; Rẽ trái gán bit '0'; Rẽ phải gán bit '1'
 */
void generateCodes(const shared_ptr<HuffmanNode>& root, 
                   const string& code, 
                   map<string, string>& codeTable) {
    if (!root) return;
    
    if (root->isLeaf()) {
        codeTable[root->symbol] = code;
        return;
    }
    
    generateCodes(root->left, code + "0", codeTable);
    
    generateCodes(root->right, code + "1", codeTable);
}

/**
 * ALGORITHM 4.4: DecodeBitStream(root, bitstream)
 * Giải mã luồng bit nhị phân liên tục thành danh sách các lệnh điều khiển Drone
 */
vector<string> decodeBitStream(const shared_ptr<HuffmanNode>& root, 
                               const string& bitstream) {
    vector<string> commands;
    auto curr = root;
    
    for (char b : bitstream) {
        if (b != '0' && b != '1') continue; // Bỏ qua ký tự không phải bit
        
        curr = (b == '0') ? curr->left : curr->right;
        
        // Khi chạm nút lá: đã khớp một từ mã tiền tố hoàn chỉnh
        if (curr && curr->isLeaf()) {
            commands.push_back(curr->symbol);
            curr = root; // Đặt lại con trỏ về gốc cây để giải mã lệnh kế tiếp
        }
    }
    return commands;
}

/**
 * Hàm hỗ trợ: Mã hóa chuỗi lệnh thành luồng bit
 */
string encodeCommands(const vector<string>& commands, const map<string, string>& codeTable) {
    string bitstream = "";
    for (const auto& cmd : commands) {
        auto it = codeTable.find(cmd);
        if (it != codeTable.end()) {
            bitstream += it->second;
        } else {
            cerr << "[WARNING] Khong tim thay ma Huffman cho lenh: " << cmd << endl;
        }
    }
    return bitstream;
}

/**
 * Hàm hỗ trợ in trực quan cấu trúc cây nhị phân dạng ASCII
 */
void printTreeASCII(const shared_ptr<HuffmanNode>& node, const string& prefix = "", bool isLeft = true) {
    if (!node) return;
    cout << prefix;
    cout << (isLeft ? "|--0-- " : "\\--1-- ");
    if (node->isLeaf()) {
        cout << "[" << node->symbol << " : " << fixed << setprecision(2) << node->freq << "]" << endl;
    } else {
        cout << "(Nút nội bộ: " << fixed << setprecision(2) << node->freq << ")" << endl;
    }
    printTreeASCII(node->left, prefix + (isLeft ? "|      " : "       "), true);
    printTreeASCII(node->right, prefix + (isLeft ? "|      " : "       "), false);
}

int main(int argc, char* argv[]) {
    // Tối ưu hóa I/O
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    cout << "======================================================================" << endl;
    cout << "CHUONG TRINH MA HOA HUFFMAN & GIAI MA TIN HIEU DIEU KHIEN DRONE" << endl;
    cout << "MAD101 - Toan Roi Rac | Mon hoc: Discrete Mathematics (FPT University)" << endl;
    cout << "======================================================================" << endl << endl;

    // 1. Tập lệnh điều khiển Drone và bảng phân bố xác suất tần suất (Mục 4.2 Báo cáo)
    vector<pair<string, double>> droneCommands = {
        {"FORWARD",  0.45},   // Bay thẳng về phía trước
        {"LEFT",     0.20},   // Rẽ trái điều hướng
        {"RIGHT",    0.15},   // Rẽ phải điều hướng
        {"HOVER",    0.10},   // Bay lơ lửng chờ định vị / tránh vật cản
        {"RECHARGE", 0.10}    // Lệnh đáp khẩn cấp vào trạm sạc
    };

    // Bảng đối chiếu mã 3-bit cố định thông thường
    map<string, string> fixedCodes = {
        {"FORWARD",  "000"},
        {"LEFT",     "001"},
        {"RIGHT",    "010"},
        {"HOVER",    "011"},
        {"RECHARGE", "100"}
    };

    // 2. Xây dựng cây Huffman (Algorithm 4.2)
    auto root = buildHuffmanTree(droneCommands);

    // 3. Sinh bảng mã tiền tố bằng duyệt tiền thứ Preorder (Algorithm 4.3)
    map<string, string> huffmanCodes;
    generateCodes(root, "", huffmanCodes);

    // 4. In Bảng 5.3: So sánh hiệu năng mã hóa Huffman vs Mã cố định 3-bit
    cout << "BANG 5.3: SO SANH MA TIEN TO HUFFMAN VS MA CO DINH 3-BIT:" << endl;
    cout << string(70, '-') << endl;
    cout << left << setw(12) << "Lenh (c)" 
         << setw(10) << "f(c)" 
         << setw(16) << "Ma 3-bit co dinh" 
         << setw(16) << "Ma Huffman" 
         << "Do dai (bits)" << endl;
    cout << string(70, '-') << endl;

    double avgHuffmanLen = 0.0;
    double avgFixedLen = 3.0; // ceil(log2(5)) = 3 bits

    for (const auto& [cmd, freq] : droneCommands) {
        string hCode = huffmanCodes[cmd];
        string fCode = fixedCodes[cmd];
        int bitLen = hCode.length();
        avgHuffmanLen += freq * bitLen;

        cout << left << setw(12) << cmd 
             << setw(10) << fixed << setprecision(2) << freq 
             << setw(16) << fCode 
             << setw(16) << hCode 
             << bitLen << " bit" << (bitLen > 1 ? "s" : " ") << endl;
    }
    cout << string(70, '-') << endl;

    // Tính toán chỉ số tiết kiệm dung lượng
    double bandwidthSaved = ((avgFixedLen - avgHuffmanLen) / avgFixedLen) * 100.0;

    cout << fixed << setprecision(2);
    cout << "-> Do dai ma trung binh per lenh (L_avg):   " << avgHuffmanLen << " bits/lenh" << endl;
    cout << "-> Do dai ma co dinh tieu chuan:           " << avgFixedLen << " bits/lenh" << endl;
    cout << "-> TY LE TIET KIEM BANG THONG VO TUYEN:    " << bandwidthSaved << "% (-30% bang thong)" << endl;
    cout << "======================================================================" << endl << endl;

    // 5. In trực quan cấu trúc Cây nhị phân đầy đủ Huffman (9 nút)
    cout << "SO DO CAY NHI PHAN MA HOA HUFFMAN (9 NUT: 5 NUT LA + 4 NUT NOI BO):" << endl;
    cout << "[Root: 1.00]" << endl;
    printTreeASCII(root->left, " ", true);
    printTreeASCII(root->right, " ", false);
    cout << "======================================================================" << endl << endl;

    // 6. Minh họa kiểm thử mã hóa và giải mã luồng bit thực tế (Algorithm 4.4)
    string testBitStream = "0101101111"; // Mẫu chuỗi bit kiểm thử chuẩn trong Báo cáo & Slide
    if (argc > 1) {
        testBitStream = argv[1];
    }

    cout << "KIEM THU MA HOA VA GIAI MA LUONG BIT (ALGORITHM 4.4):" << endl;
    cout << "----------------------------------------------------------------------" << endl;
    vector<string> sampleCommands = {"FORWARD", "LEFT", "RIGHT", "RECHARGE"};
    cout << "1. Chuoi lenh dau vao can truyen:" << endl;
    cout << "   [ ";
    for (size_t i = 0; i < sampleCommands.size(); ++i) {
        cout << sampleCommands[i] << (i + 1 < sampleCommands.size() ? ", " : " ");
    }
    cout << "]" << endl;

    string encodedBits = encodeCommands(sampleCommands, huffmanCodes);
    cout << "2. Luong bit sau ma hoa Huffman:  \"" << encodedBits << "\" (" << encodedBits.length() << " bits)" << endl;
    cout << "   (Neu dung ma co dinh 3-bit:    4 lenh * 3 = 12 bits -> tiet kiem 2 bits)" << endl << endl;

    cout << "3. Giai ma luong bit nhan duoc:   \"" << testBitStream << "\"" << endl;
    vector<string> decodedCommands = decodeBitStream(root, testBitStream);
    cout << "   Ket qua giai ma tren Drone:" << endl;
    for (size_t i = 0; i < decodedCommands.size(); ++i) {
        cout << "   + Buoc " << (i + 1) << ": Khop tu ma \"" 
             << huffmanCodes[decodedCommands[i]] << "\" -> LENH: " 
             << decodedCommands[i] << endl;
    }
    cout << "======================================================================" << endl;

    return 0;
}
