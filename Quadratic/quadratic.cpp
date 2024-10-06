#include <iostream>
#include <cmath>
using namespace std;



int quadsolver(int a, int b, int c){
    double x1;
    double x2;
    double discriminant = sqrt(b*b - 4*a*c);

    if(discriminant >= 0){
     x1 = (-b + sqrt(b*b - 4*a*c))/(2*a);
     x2 = (-b - sqrt(b*b - 4*a*c))/(2*a);
    }

    cout << "The solutions are " << x1 << " and " << x2 << "\n"; 
    return 0;
}
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

    quadsolver(a,b,c);
}