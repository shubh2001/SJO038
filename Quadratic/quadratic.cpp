#include <iostream>
#include <cmath>
using namespace std;

int main(){
    int a;
    int b;
    int c;

    cout << "Enter value of a: \n";
    cin >> a;

    cout << "Enter value of b: \n";
    cin >> b;

    cout << "Enter value of c: \n";
    cin >> c;

    double x1 = (-b + sqrt(b*b - 4*a*c))/(2*a);
    double x2 = (-b - sqrt(b*b - 4*a*c))/(2*a);

   
    cout << "The solutions are " << x1 << " and " << x2 << "\n"; 
}