//#include<string>
//#include<vector>
//#include<iostream>
//#include<stack>
//#include<queue>
//#include<unordered_map>
//#include<set>
//using namespace std;
//class Solution0 {
//public:
//	int removeElement(vector<int>& nums, int val) {
//		int i;
//		for (i = 0; i < nums.size(); i++)
//		{
//			if (nums[i] == val)
//			{
//				int j;
//				for (j = i + 1; j < nums.size(); j++)
//				{
//					if (nums[j] != val)
//					{
//						swap(nums[i], nums[j]);
//						break;
//					}
//
//				}
//				if (j == nums.size())
//					break;
//			}
//		}
//		return i;
//	}
//
//private:
//	void swap(int &t1, int &t2)
//	{
//		int temp = t1;
//		t1 = t2;
//		t2 = temp;
//	}
//};
//
//class Solution {
//public:
//	string reverseVowels(string s)
//	{
//		int i = 0;
//		int j = s.size() - 1;
//		while (true)
//		{
//			while (!isVowel(s[i]))
//			{
//				i++;
//			}
//			while (!isVowel(s[j]))
//			{
//				j--;
//			}
//			if (i >= j)
//				break;
//			swap(s[i], s[j]);
//			i++;
//			j--;
//		}
//		return s;
//
//	}
//	
//private:
//	void swap(char &c1, char &c2)
//	{
//		char tmp = c1;
//		c1 = c2;
//		c2 = tmp;
//	}
//	bool isVowel(const char &letter)
//	{
//		return letter == 'a' || letter == 'e' || letter == 'i' || letter == 'o' || letter == 'u';
//	}
//};
//
//class Solution1 {
//public:
//	bool validPalindrome(string s) {
//		int i=0;
//		int j = s.size() - 1;
//		while (i < j)
//		{
//			if (s[i] == s[j])
//			{
//				i++;
//				j--;
//			}
//			else if (s[i] == s[j - 1])
//			{
//				i++;
//				j = j - 2;
//			}
//			else if (s[i + 1] == s[j])
//			{
//				i = i + 2;
//				j--;
//			}
//			else
//				return false;
//		}
//		return true;
//	}
//};
//class Solution2{
//public:
//	string findLongestWord(string s, vector<string>& d) {
//		vector<int> index;
//		size_t maxIndex = 0;
//		for (int i = 0; i < d.size(); i++)
//		{
//			if (isSubstring(s, d[i]))
//				index.push_back(i);
//		}
//		if (index.empty()) return "";
//		for (int i = 0; i < index.size(); i++)
//		{
//			if (maxIndex < d[index[i]].size())
//				maxIndex = index[i];
//		}
//		return d[maxIndex];
//	}
//private:
//	bool isSubstring(string s1, string s2)
//	{
//		int i = 0, j = 0;
//		while (i < s1.size() && j < s2.size())
//		{
//			if (s1[i] == s2[j])
//			{
//				i++;
//				j++;
//			}
//			else i++;
//		}
//		if (j == s2.size())
//			return true;
//		else
//			return false;
//	}
//};
//class Solution3 {
//public:
//	int maxProfit(vector<int>& prices) {
//		int min = prices[0];
//		int total = 0;
//		int profit = 0;
//		bool isfirst = false;
//
//		for (int i = 1; i < prices.size(); ++i)
//		{
//			if (prices[i] - prices[i-1] >= 0)
//			{
//				profit = prices[i] - min;
//				isfirst = true;
//			}
//
//			else
//			{
//				min = prices[i];
//				if (isfirst)
//					total += profit;
//				isfirst = false;
//			}
//		}
//		return total;
//	}
//};
//class Solution4 {
//public:
//	bool canPlaceFlowers(vector<int>& flowerbed, int n) {
//		flowerbed.insert(flowerbed.begin(), 0);
//		flowerbed.push_back(0);
//		int total = 0;
//		for (int i = 1; i < flowerbed.size() - 1; ++i)
//			if (flowerbed[i - 1] ==0&& flowerbed[i] ==0&& flowerbed[i + 1] == 0)
//			{
//				flowerbed[i] = 1;
//				++total;
//			}
//		return total > n;
//
//	
//	}
//};
//struct ListNode {
//	int val;
//	ListNode *next;
//	ListNode(int x) : val(x), next(NULL) {}
//	
//};
//class Solution6 {
//public:
//	ListNode* reverseList(ListNode* head) {
//		if (head == NULL) return NULL;
//		ListNode* A = new ListNode(*head);
//		ListNode* now;
//		while (head->next)
//		{
//			now = head->next;
//			head->next = now->next;
//
//			now->next = A->next;
//			A->next = now;
//
//		}
//		return A;
//	}
//};
//
//class Solution7 {
//public:
//	vector<ListNode*> splitListToParts(ListNode* root, int k) {
//		vector<int> vec;
//		ListNode* now = root;
//		int count = 0;
//		while (now)
//		{
//			now = now->next;
//			++count;
//		}
//		int s1 = count / k; int s2 = count % k;
//		for (int i = 0; i < k; ++i)
//		{
//			if (s2 > 0)
//			{
//				vec.push_back(s1 + 1);
//				--s2;
//			}
//			else
//				vec.push_back(s1);
//		}
//
//		vector<ListNode*> result;
//		/*ListNode* sentry = new ListNode(-1);
//		sentry->next = root;
//		ListNode* pre = sentry;*/
//		now = root;
//		ListNode* newnow = new ListNode(now->val);
//		for (auto number : vec)
//		{
//			result.push_back(newnow);
//			for (int i = 0; i < number; ++i)
//			{
//				now = now->next;
//				newnow->next = new ListNode(now->val);
//			}
//			
//		}
//		return result;
//	}
//};
//class Solution8{
//public:
//	bool isAnagram(string s, string t) {
//		if (s.size() != t.size()) return false;
//		vector<int> svec(26, 0);
//		vector<int> tvec(26, 0);
//		for (char c : s)
//			++svec[c - 'a'];
//		for (char d : s)
//			++tvec[d - 'a'];
//		for (int i = 0; i < svec.size(); ++i)
//			if (svec[i] != tvec[i]) return false;
//
//		return true;
//
//	}
//};
//int main()
//{
//	string s = "a";
//	string t = "b";
//	Solution8 obj;
//	obj.isAnagram(s, t);
//	system("pause");
//	return 0;
//}