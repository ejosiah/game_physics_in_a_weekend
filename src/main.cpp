#include <iostream>

#include "GameWorld.hpp"

int main(){
    fs::current_path("../../resources");
    try{
        Settings settings;
        settings.depthTest = true;

        auto app = GameWorld{ settings };
        app.run();
    }catch(std::runtime_error& err){
        spdlog::error(err.what());
        return 500;
    }
    return 0;
}
