#include <drogon/drogon.h>

int main() {
  drogon::app().addListener("127.0.0.1", 8848).run();
  return 0;
}