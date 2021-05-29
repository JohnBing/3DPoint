//#include <iostream>
//#include <string>
//#include <boost/shared_ptr.hpp>
//#include <vector>
//#include<utility>
//using namespace std;
//
//class Book
//{
//private:
//	string name_;
//
//public:
//	Book(string name) : name_(name)
//	{
//		cout << "Creating book " << name_ << " ..." << endl;
//	}
//
//	~Book()
//	{
//		cout << "Destroying book " << name_ << " ..." << endl;
//	}
//};
//
//int main()
//{
//	cout << "=====Main Begin=====" << endl;
//	{
//		boost::shared_ptr<Book> myBook(new Book("「1984」"));
//		cout << "[From myBook] The ref count of book is " << myBook.use_count() << ".\n" << endl;
//
//		boost::shared_ptr<Book> myBook1(myBook);
//		cout << "[From myBook] The ref count of book is " << myBook.use_count() << "." << endl;
//		cout << "[From myBook1] The ref count of book is " << myBook1.use_count() << ".\n" << endl;
//
//		cout << "Reset for 1th time. Begin..." << endl;
//		myBook.reset();
//		cout << "[From myBook] The ref count of book is " << myBook.use_count() << "." << endl;
//		cout << "[From myBook1] The ref count of book is " << myBook1.use_count() << "." << endl;
//		cout << "Reset for 1th time. End ...\n" << endl;
//
//		cout << "Reset for 2th time. Begin ..." << endl;
//		myBook1.reset();
//		cout << "Reset for 2th time. End ..." << endl;
//	}
//	cout << "===== Main End =====" << endl;
//	system("pause");
//	return 0;
//}
//
//
////#include <iostream>
////#include <memory>
////using namespace std;
////
////void Check(weak_ptr<int> &wp)
////{
////	shared_ptr<int> sp = wp.lock(); // 重新获得shared_ptr对象
////	if (sp != nullptr)
////	{
////		cout << "The value is " << *sp << endl;
////	}
////	else
////	{
////		cout << "Pointer is invalid." << endl;
////	}
////}
////
////int main()
////{
////	/*const int a = 10;
////	int b=const_cast<int>(a);*/
////	//int a = 10;
////	//char b = (char)a;                     //c风格转换
////	//char c = static_cast<char>(a);        //c++风格转换
////	const int a = 10;
////	int b = (int)a;                    //C语言类型转换
////	int c = const_cast<int>(a);        //C++类型转换   
////
////	shared_ptr<int> sp1(new int(10));
////	cout<<sp1.use_count();
////	shared_ptr<int> sp2 = sp1;
////	cout << sp1.use_count();
////	weak_ptr<int> wp = sp1; // 指向sp1所指向的内存
////
////	cout << *sp1 << endl;
////	cout << *sp2 << endl;
////	Check(wp);
////
////	sp1.reset();
////	cout << *sp2 << endl;
////	Check(wp);
////
////	sp2.reset();
////	Check(wp);
////
////	system("pause");
////	return 0;
////}