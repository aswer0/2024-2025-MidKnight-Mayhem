//#include <iostream>
//#include <utility>
//#include <fstream>
//#include <iostream>
//#include <vector>
//#include <algorithm>
//#include <utility>
//#include <set>
//#include <string>
//#include <queue>
//#include <map>
//#include <climits>
//#include <iomanip>
//
//using namespace std;
//
//bool cmp(const pair<long long, long long> &x, const pair<long long, long long> &y) {
//    return x.second > y.second || (x.second == y.second && x.first < y.first);
//}
//
//int main() {
//    // freopen("convention.in", "r", stdin);
//    // freopen("convention.out", "w", stdout);
//
//    long long t;
//
//    cin >> t;
//
//    while (t--){
//        long long n;
//        long long ans = LLONG_MAX;
//
//        cin >> n;
//
//        vector<pair<long long, long long> > gifts(n);
//        multiset<long long> ms;
//
//        for (long long i=0; i<n; i++){
//            cin >> gifts[i].first >> gifts[i].second;
//            ms.insert(gifts[i].first);
//        }
//
//        sort(gifts.begin(), gifts.end(), cmp);
//
//        for (int i=1; i<n; i++){
//            ans = min(ans, abs(gifts[i].first-gifts[0].second));
//        }
//
//        long long max_a;
//        for (long long i=0; i<n; i++){
//            long long ai = gifts[i].first;
//            long long bj = gifts[i].second;
//
//            ms.erase(ms.find(ai));
//
//            if (i == 0){
//                max_a = ai;
//                continue;
//            }
//
//            ans = min(ans, abs(max_a-bj));
//
//            auto it = ms.lower_bound(bj);
//            if (it != ms.end() && *it >= max_a){
//                ans = min(ans, abs(*it-bj));
//            }
//            if (it != ms.begin()){
//                if (*prev(it) >= max_a){
//                    ans = min(ans, abs(*prev(it)-bj));
//                }
//            }
//
//            max_a = max(max_a, ai);
//
//        }
//
//        cout << ans << endl;
//
//    }
//
//}
