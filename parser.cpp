#include <bits/stdc++.h>
using namespace std;

int main()
{
    freopen("input/linearbatchsam.txt","r",stdin);
    freopen("output/linearbatchsam.txt","w",stdout);
    string s;
    //cout << "initial graph:\n";
    for(int i=0;i<95;i++){
        getline(cin,s);
        istringstream ss(s);
        string token;
        int cnt=1;
        while(getline(ss,token,' ')){
            if(cnt==3){
                token.pop_back();
                reverse(token.begin(),token.end());
                token.pop_back();
                reverse(token.begin(),token.end());
                cout << token << " ";
            }
            else if(cnt==4){
                token.pop_back();
                cout << token << endl;
            }
            cnt++;
        }
    }
    //cout << "initial landmarks:\n";
    for(int i=0;i<24;i++){
        getline(cin,s);
        istringstream ss(s);
        string token;
        int cnt=1;
        while(getline(ss,token,' ')){
            if(cnt==3){
                token.pop_back();
                reverse(token.begin(),token.end());
                token.pop_back();
                reverse(token.begin(),token.end());
                cout << token << " ";
            }
            else if(cnt==4){
                token.pop_back();
                cout << token << endl;
            }
            cnt++;
        }
    }

    //cout << "final graph:\n";
    for(int i=0;i<95;i++){
        getline(cin,s);
        istringstream ss(s);
        string token;
        int cnt=1;
        while(getline(ss,token,' ')){
            if(cnt==3){
                token.pop_back();
                reverse(token.begin(),token.end());
                token.pop_back();
                reverse(token.begin(),token.end());
                cout << token << " ";
            }
            else if(cnt==4){
                token.pop_back();
                cout << token << endl;
            }
            cnt++;
        }
    }
    //cout << "final landmarks:\n";
    for(int i=0;i<24;i++){
        getline(cin,s);
        istringstream ss(s);
        string token;
        int cnt=1;
        while(getline(ss,token,' ')){
            if(cnt==3){
                token.pop_back();
                reverse(token.begin(),token.end());
                token.pop_back();
                reverse(token.begin(),token.end());
                cout << token << " ";
            }
            else if(cnt==4){
                token.pop_back();
                cout << token << endl;
            }
            cnt++;
        }
    }
    return 0;
}
