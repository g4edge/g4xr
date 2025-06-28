#include "G4XrViewer.hh"
#include "G4VSceneHandler.hh"
#include "G4XrSceneHandler.hh" //BEN

// tinygltf
// Define these only in *one* .cc file.
//BEN - MOVED THE COMMENTED TO XRSCENEHANDLER
//#define TINYGLTF_IMPLEMENTATION
//#define STB_IMAGE_IMPLEMENTATION
//#define STB_IMAGE_WRITE_IMPLEMENTATION
// #define TINYGLTF_NOEXCEPTION // optional. disable exception handling.

namespace fs = std::filesystem;

G4XrViewer::G4XrViewer(G4VSceneHandler& sceneHandler, const G4String& name)
  : G4VViewer(sceneHandler, sceneHandler.IncrementViewCount(), name)
{
    
  // Set default and current view parameters
  fVP.SetAutoRefresh(true);
  fDefaultVP.SetAutoRefresh(true);

}

void G4XrViewer::Initialise() // server code - Ben (05/27/2025)
{
  std::cout << "G4XrViewer::Initialise()" << std::endl;
  /*svr.Get("/hi", [](const httplib::Request &, httplib::Response &res) {
    res.set_content("Hello World!", "text/plain");
  });

    std::cout<<"Initialized Server"<<std::endl;

  svr_thread = std::thread([this]() {
    this->svr.listen("0.0.0.0", 8080);  // This blocks inside the thread
  });*/ // - Stewart's Implementation; "I'm pushing the server launch to an outside method so that this one is cleaner" - Ben 05/27/2025
    
    server_init();

}



G4XrViewer::~G4XrViewer()
{
}

void G4XrViewer::SetView()
{
}

void G4XrViewer::DrawView()
{
  NeedKernelVisit();

  ProcessView();

  FinishView();
        
}

void G4XrViewer::ShowView() {
}

void G4XrViewer::ClearView()
{
}

void G4XrViewer::FinishView()
{
    // BEN - casting issues.
    /*std::cout<<"Pushing mesh data to GLTF..."<<std::endl;
    auto* xrHandler = dynamic_cast<G4XrSceneHandler*>(fSceneHandler);
    const auto& meshes = xrHandler->GetCollectedMeshes();
    ConvertMeshToGLTF(meshes, "trial.gltf");*/
}

// Ben - 05/27/2025
int G4XrViewer::server_init() // this is adapted server code that was previously used to test G4VR's web requesting functionality. Some elements are not necessary for a basic local server and can be removed for conciseness. - BEN
{
    if (fs::exists(UPLOAD_DIR)) { // cleaning code
            fs::remove_all(UPLOAD_DIR);
        }
    
    fs::create_directories(UPLOAD_DIR);

    svr.Post(R"(/upload/(\w+))", [&](const httplib::Request& req, httplib::Response& res)
    {
        std::string userId = req.matches[1];
        const auto& files = req.files;

        for (auto& [key, file] : files) {

            fs::path user_path = fs::path(UPLOAD_DIR) / userId;
            fs::create_directories(user_path);

            fs::path file_path = user_path / file.filename;
            std::ofstream ofs(file_path, std::ios::binary);
            //ofs << file.content; truncating? - BEN 06/02/2025
            ofs.write(file.content.data(), file.content.size());
            //ofs.close();
        }

        res.set_content("Upload Complete", "text/plain");
    });

    svr.Get(R"(/files/(\w+)/list)", [&](const httplib::Request& req, httplib::Response& res)
    {
        std::string userId = req.matches[1];
        fs::path user_path = fs::path(UPLOAD_DIR) / userId;

        std::string json = "[";
        bool first = true;
        for ( auto& entry : fs::directory_iterator(user_path)) {
            if (entry.is_regular_file()) {
                if (!first) json += ",";
                json += "\"" + entry.path().filename().string() + "\"";
                first = false;
            }
        }
        json += "]";
        res.set_content(json, "application/json");
    });

    svr.Get(R"(/files/(\w+)/(.+))", [&](const httplib::Request& req, httplib::Response& res) {
        std::string userId = req.matches[1];
        std::string filename = req.matches[2];

        fs::path file_path = fs::path(UPLOAD_DIR) / userId / filename;


        auto ifs = std::make_shared<std::ifstream>(file_path, std::ios::binary);
        
        auto file_size = fs::file_size(file_path);
        res.set_header("Content-Length", std::to_string(file_size));

        res.set_content_provider(
            "application/octet-stream",
            [file_path, file_size](size_t offset, httplib::DataSink &sink) {
                if (offset >= file_size) {
                    return false;
                }

                size_t chunk_size = 8192;
                size_t to_read = std::min(chunk_size, file_size - offset);
                std::vector<char> buffer(to_read);

                std::ifstream ifs(file_path, std::ios::binary);
                if (!ifs) return false;

                ifs.seekg(offset, std::ios::beg);
                ifs.read(buffer.data(), to_read);
                size_t bytes_read = static_cast<size_t>(ifs.gcount());

                if (bytes_read > 0) {
                    sink.write(buffer.data(), bytes_read);
                    return true;
                }

                return false;
            },
            [](bool success) {
                // no resource to clean here
            }
        );





    });

    std::string local_ip = get_local_ip();
    URL = "http://" + local_ip + ":"+std::to_string(PORT);
    G4cout << "Enter this address in G4VR: http://" << local_ip << ":" << PORT << G4endl;
    
    svr_thread = std::thread([this]() {
            svr.listen("0.0.0.0", PORT);
        });
        svr_thread.detach();

        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::thread([this]() {while (!file_pushed) {push_file();} }).detach();

    return 0;
}

// Ben - 05/27/2025
std::string G4XrViewer::get_local_ip()
{
    std::string local_ip = "127.0.0.1";
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    //if (sock < 0){perror("socket"); return local_ip;}
    
    sockaddr_in serv;
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = inet_addr("8.8.8.8");
    serv.sin_port = htons(53);

    int err = connect(sock, (const sockaddr*)&serv, sizeof(serv));
    if (err < 0)
    {
        perror("connect");
        close(sock);
        return local_ip;
    }
    sockaddr_in name;
    socklen_t namelen = sizeof(name);
    err = getsockname(sock, (sockaddr*)&name, &namelen);
    if (err < 0) {
        perror("getsockname");
        close(sock);
        return local_ip;
    }
    char buffer[INET_ADDRSTRLEN];
    const char* p = inet_ntop(AF_INET, &name.sin_addr, buffer, sizeof(buffer));
    if (p != nullptr) {
        local_ip = buffer;
    }

    close(sock);
    return local_ip;
}

// Ben - 05/27/2025
void G4XrViewer::push_file(const std::string& dirname) { // modify if renamed later - BEN - this is looking for a directory containing gltf meta-data. a good approach would be to create a gltf directory in build which stores all the data.
    httplib::Client cli(URL.c_str());
    
    try
    {
        for (const auto& entry : std::filesystem::directory_iterator(std::filesystem::current_path().c_str()+dirname)) {
            std::cout<<"Pushing from "<<std::filesystem::current_path().c_str()+dirname<<std::endl;
            if (entry.is_regular_file()) {
                auto filepath = entry.path();
                std::ifstream ifs(filepath, std::ios::binary);
                std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
                
                httplib::MultipartFormDataItems items = {
                    { "file", content, filepath.filename().string(), "application/octet-stream" }
                };
                auto res = cli.Post(("/upload/testuser"), items); // "testuser" is arbitrary as long as it is consistent with where G4VR looks...
                if (!res)
                {std::cerr << "Server Connection Lost\n"; return;}
                file_pushed = true;
            }
        }
    } catch (...){//std::cout<<"file not found - caught";
    }
}




