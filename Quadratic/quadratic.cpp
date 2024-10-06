#include <iostream>
#include <cmath>
using namespace std;



int quadsolver(int a, int b, int c){
    double x1;
    double x2;
    double discriminant = sqrt(b*b - 4*a*c);

    if(discriminant > 0){
     x1 = (-b + sqrt(b*b - 4*a*c))/(2*a);
     x2 = (-b - sqrt(b*b - 4*a*c))/(2*a);
    }
    else if (discriminant == 0) {
     x1 = -b/(2*a);
     x2 = x1;
     cout << "The single solution is " << x1 << "\n"; 
     return 0;
    }
    else{
        double imaginary = sqrt(4*a*c-b*b)/2;
        cout << "This quadratic equation has complex solutions: " << -b/2 << "+" << imaginary << "i and " << -b/2 << "-" << imaginary << "i \n";
        return 0;
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