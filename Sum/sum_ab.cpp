#include <iostream>
using namespace std;

int main(){
    int num1;
    int num2;
    int sum = 0;

    cout << "Enter two numbers \n";
    cin >> num1;
    cin >> num2;
    
    for(int i=num1; i<= num2; i++){
        sum += i;
    }
    cout << "Sum of Numbers between " << num1 << " and " << num2 << " is " << sum << "\n";
}