#include <drogon/HttpAppFramework.h>
#include <drogon/drogon.h>
#include "handler/rest.cpp"

int main() {
  std::cout << "server started at http://127.0.0.1:8848" << std::endl;
  drogon::app().addListener("127.0.0.1", 8848).run();
  return 0;
}
