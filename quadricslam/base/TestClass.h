
namespace gtsam {

class TestClass {
  private:
    double x_;

  public:
    TestClass() {};
    TestClass(double x) : x_(x) {};
    double getx(void) {return x_;}
    double doWork(double y);
};

}