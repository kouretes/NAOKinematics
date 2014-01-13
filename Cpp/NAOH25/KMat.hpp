#ifndef __KMAT_H__
#define __KMAT_H__
#include <iostream>
#include <stdexcept>
#include <string.h>
#include <iomanip>
#include <limits>
#include <math.h>

#ifdef NDEBUG
#define KMAT_INSANE_MODE
#endif

#ifdef UNUSED
#elif defined(__GNUC__)
#define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSED(x) /*@unused@*/ x
#else
# define UNUSED(x) x
#endif

#ifdef __RESTRICTED
#elif defined(__GNUC__)
#define __RESTRICTED __restrict__
#else
#define __RESTRICTED
#endif
/**
 * KMat (koimat`) :: Kouretes Matrix Library!
 * Provides a templated statically binded (sounds exotic :P )
 * matrix library
 * optimized for small matrices (ie NOT optimized for large matrices)
 *
 * @author vosk
 * */
namespace KMath {
namespace KMat
{
	using std::runtime_error;
	class MatrixIndexOutOfBoundsException : public std::runtime_error
	{
	public:
		MatrixIndexOutOfBoundsException(std::string m = "") :	runtime_error(m.append( ": MatrixIndexOutOfBoundsException : attempted to access an invalid element") ) {}
	};
	class SingularMatrixInvertionException : public std::runtime_error
	{
	public:
		SingularMatrixInvertionException(std::string m = "") :	runtime_error(m.append( ": SingularMatrixInvertionException : attempted to invert a singular matrix") ) {}
	};

	template<typename T, typename C> class COWRef
	{
		public:
		 COWRef(C &obj,int a, int b): p(obj),i(a),j(b){};
		 //const AT& operator(){ return p.getFunc(i,j) ;} const;
		 T& operator=(T const  v) {return  p.get(i,j)=v;};
		 T& operator=(COWRef<T,C>  const &v) { return p.get(i,j)=v.p.read(v.i,v.j);};
		 template<typename AT,typename AC> T operator=(COWRef<AT,AC> const& v) { return p.get(i,j)=v.p.read(v.i,v.j);};
		 operator T() const
		 {
		 	return p.read(i,j);
		 }
		 private:
		 C & p;
		 int i,j;
	};

	template<typename C> class COWRef<float, C>
	{
		//template <typename AT, typename AC>	friend class COWRef;
	public:
		COWRef(C &obj, int a, int b): p(obj), i(a), j(b) {};
		operator float() const
		{
			return p.read(i, j) ;
		} ;
		//template<typename AT,typename AC> float operator=(COWRef<AT,AC> const& v) { return p.get(i,j)=v.p.read(v.i,v.j);};
		float operator=(COWRef<float, C>  const &v)
		{
			return p.get(i, j) = v.p.read(v.i, v.j);
		};
		float operator=(float  v)
		{
			return p.get(i, j) = v;
		};
		template<typename AT> float operator+=(AT  v)
		{
			return p.get(i, j) += v;
		};
		template<typename AT> float operator-=(AT  v)
		{
			return p.get(i, j) -= v;
		};
		template<typename AT> float operator*=(AT  v)
		{
			return p.get(i, j) *= v;
		};
		template<typename AT> float operator/=(AT  v)
		{
			return p.get(i, j) /= v;
		};

	private:
		C & p;
		const int i, j;
	};

	template<typename C> class COWRef<double, C>
	{
		//template <typename AT, typename AC>	friend class COWRef;
	public:
		COWRef(C &obj, int a, int b): p(obj), i(a), j(b) {};
		operator double() const
		{
			return p.read(i, j) ;
		} ;
		//template<typename AT,typename AC> float operator=(COWRef<AT,AC> const& v) { return p.get(i,j)=v.p.read(v.i,v.j);};
		double operator=(COWRef<double, C>  const &v)
		{
			return p.get(i, j) = v.p.read(v.i, v.j);
		};
		double operator=(double  v)
		{
			return p.get(i, j) = v;
		};
		template<typename AT> double operator+=(AT  v)
		{
			return p.get(i, j) += v;
		};
		template<typename AT> double operator-=(AT  v)
		{
			return p.get(i, j) -= v;
		};
		template<typename AT> double operator*=(AT  v)
		{
			return p.get(i, j) *= v;
		};
		template<typename AT> double operator/=(AT  v)
		{
			return p.get(i, j) /= v;
		};

	private:
		C & p;
		const int i, j;
	};

	template<typename C> class COWRef<int, C>
	{
		//template <typename AT, typename AC>	friend class COWRef;
	public:
		COWRef(C &obj, int a, int b): p(obj), i(a), j(b) {};
		operator int() const
		{
			return p.read(i, j) ;
		} ;
		//template<typename AT, typename AC> int operator=(COWRef<AT,AC> const & v) { return p.get(i,j)=v.p.read(v.i,v.j);};
		int operator=(COWRef<int, C>  const &v)
		{
			return p.get(i, j) = v.p.read(v.i, v.j);
		};
		int operator= (int  v)
		{
			return p.get(i, j) = v;
		};
		template<typename AT> int operator+=(AT  v)
		{
			return p.get(i, j) += v;
		};
		template<typename AT> int operator-=(AT  v)
		{
			return p.get(i, j) -= v;
		};
		template<typename AT> int operator*=(AT  v)
		{
			return p.get(i, j) *= v;
		};
		template<typename AT> int operator/=(AT  v)
		{
			return p.get(i, j) /= v;
		};

	private:
		C & p;
		const int i, j;
	};

	template <template<typename , unsigned , unsigned > class	D, typename T, unsigned M, unsigned N>
	class BaseMatrix;

	template <typename AT, unsigned AM , unsigned AN> class DataContainer
	{
	private:
		AT mem[AM][AN];

	protected:

		template <template<typename , unsigned , unsigned > class	D, typename T, unsigned M, unsigned N>
		friend class BaseMatrix;


		inline AT & data(unsigned i, unsigned j)
		{
			return mem[i][j];
		};
		inline AT *data(unsigned i)
		{
			return mem[i];
		};
		void zero()
		{
			memset(mem , 0 , sizeof(mem));
		};
	public:
		DataContainer() {};

	};

	template<typename AT> class DataContainer<AT, 1, 1>
	{

	public:
		AT x;
	protected:
		template <template<typename , unsigned , unsigned > class	D, typename T, unsigned M, unsigned N>
		friend class BaseMatrix;

		inline AT & data(unsigned UNUSED(i), unsigned  UNUSED(j) )
		{
			return x;
		};
		inline AT * data(unsigned UNUSED(i))
		{
			return &x;
		};
		void zero()
		{
			x = 0;
		};
	};
	template<typename AT> class DataContainer<AT, 2, 1>
	{
	public:
		AT x, y;


	protected:
		template <template<typename , unsigned , unsigned > class	D, typename T, unsigned M, unsigned N>
		friend class BaseMatrix;

		inline AT & data(unsigned i, unsigned UNUSED(j) )
		{
			switch(i)
			{
			case 0:
				return x;

			default:
				return y;
			};
		};

		inline AT * data(unsigned i)
		{
			switch(i)
			{
			case 0:
				return &x;

			default:
				return &y;
			};
		};
		void zero()
		{
			x = 0;
			y = 0;
		};
	};

	template<typename T> class RefCounted : public T
	{
	private:
		unsigned counts;
	public:
		RefCounted() : T(), counts(0) {};
		RefCounted(RefCounted<T> const & r): T(r), counts(0) {};
		RefCounted<T> & operator=(RefCounted<T> const & c)
		{
			T::operator=(c);
			counts = 0;
			return *this;
		}
		void inc()
		{
			++counts;
		};
		void dec()
		{
			--counts;
		};
		bool isExclusive() const
		{
			return counts <= 1;
		};

	};
	template<typename T> class RefHandle
	{
	protected:
		RefCounted<T> * __RESTRICTED h;
		RefHandle() : h(0) {};
		RefHandle(RefHandle<T> const & o)
		{
			h = o.h;

			if(h != NULL)
				h->inc();
		}
		RefHandle<T> & operator=(RefHandle<T> const& p)
		{
			if(h == p.h)
				return *this;

			cleanHandle();
			h = p.h;

			if(h != NULL)
				h->inc();

			return *this;
		}
		~RefHandle()
		{
			cleanHandle();
		};
		void cleanHandle()
		{
			if(h == NULL)
				return;

			if(h->isExclusive())
				delete h;
			else
				h->dec();

			h = NULL;
		}
		void inc()
		{
			h->inc();
		};
		void dec()
		{
			h->dec();
		};
		bool validHandle() const
		{
			return h != NULL;
		}
		bool getHandle() //Return true if a new object has been created
		{
			if(h == NULL)
			{
				h = new RefCounted<T>();
				h->inc();
				return true;
			}

			//std::cout<<"jc:"<<h->counts<<std::endl;
			if(h->isExclusive())
				return false;

			h->dec();
			h = new RefCounted<T>(*h);
			h->inc();
			return false;
		}
	};
	template<typename T> class LoopBackHandle :  public T
	{
	protected:
		T *const h;
		LoopBackHandle() : T(), h(this) {};
		LoopBackHandle(LoopBackHandle<T> const &a ) : T(a), h(this)
		{		};
		LoopBackHandle<T> & operator=(LoopBackHandle<T> const & o)
		{
			T::operator=(o);
			return *this;
		}
		inline static void cleanHandle()
		{
		}
		inline static void inc()  {};
		inline static void dec()  {};
		inline static bool validHandle()
		{
			return  true;
		};
		inline static bool getHandle() //Return true if a new object has been created
		{
			return false;
		}
	};
	//Use LoopBackHandle to implement COW-less 1x1 2x1 3x1 RefHandles
	template<typename T> class RefHandle<DataContainer<T, 1, 1> > : public LoopBackHandle<DataContainer<T, 1, 1> >
{};
	template<typename T> class RefHandle<DataContainer<T, 2, 1> > : public LoopBackHandle<DataContainer<T, 2, 1> >
{};
template<typename T> class RefHandle<DataContainer<T, 3, 1> > : public LoopBackHandle<DataContainer<T, 3, 1> >
{};
	/*
	 * Base class, utilizes the CRTP idiom to provide a nice way of implementing static polymorphism
	 *
	 */
	template <template<typename , unsigned , unsigned > class	D, typename T, unsigned M, unsigned N>
	class BaseMatrix : public RefHandle<DataContainer<T, M, N> >
	{
		template <template<typename , unsigned , unsigned > class	AD, typename AT, unsigned AM, unsigned AN>	friend class BaseMatrix;

	protected:
		using RefHandle<DataContainer<T, M, N> >::h;



	public:
		BaseMatrix() {};
		BaseMatrix(BaseMatrix<D, T, M, N> const& o) : RefHandle<DataContainer<T, M, N> >(o)
		{
			//if(h!=NULL)
			//	RefCounter<DataContainer<T,M,N> >::inc();
		}
		bool isInitialized()
		{
			return h != 0;
		};
		typedef std::numeric_limits<T> Tlimits;

		//============================	BASIC FUNCTIONS ============================
		/**
		 *	In place add another matrix to this
		 **/
		D<T, M, N>& operator+=( BaseMatrix<D, T, M, N> const& rop)
		{
			return add(rop);
		}
		D<T, M, N>& add( BaseMatrix<D, T, M, N> const& __RESTRICTED rop)
		{
			if(h == NULL || rop.h == NULL)
				return static_cast< D<T, M, N> &> (*this);

			RefHandle<DataContainer<T, M, N> >::getHandle();

			for (unsigned i = 0; i < M; i++)
				for (unsigned j = 0; j < N; j++)
					h->data(i, j) += rop.h->data(i, j);

			return static_cast< D<T, M, N> &> (*this);
		};

		D<T, M, N> operator+( BaseMatrix<D, T, M, N> const& rop)
		{
			D<T, M, N> res;
			res=(*this);
			res+=rop;
			return res;
		}

		/**
		 *	In place add another matrix to this, column wise add
		 **/
		D<T, M, N>& column_add( BaseMatrix<D, T, M, 1> const& __RESTRICTED rop)
		{
			if(h == NULL || rop.h == NULL)
				return static_cast< D<T, M, N> &> (*this);

			RefHandle<DataContainer<T, M, N> >::getHandle();

			for (unsigned i = 0; i < M; i++)
				for (unsigned j = 0; j < N; j++)
					h->data(i, j) += rop.h->data(i, 0);

			return static_cast< D<T, M, N> &> (*this);
		};


		/**
		 *	Swap columns of matrix
		 **/
		D<T, M, N>& column_swap( unsigned k, unsigned l)
		{
			if(h == NULL)
				return static_cast< D<T, M, N> &> (*this);

			RefHandle<DataContainer<T, M, N> >::getHandle();

			for (unsigned i = 0; i < M; i++)
			{
				T temp=h->data(i,k);
				h->data(i,k)=h->data(i,l);
				h->data(i,l)=temp;

			}

			return static_cast< D<T, M, N> &> (*this);
		};
		/**
		 *	In place add another matrix to this, row wise add
		 **/
		D<T, M, N>& row_add( BaseMatrix<D, T, 1, N> const& rop)
		{
			if(h == NULL || rop.h == NULL)
				return static_cast< D<T, M, N> &> (*this);

			RefHandle<DataContainer<T, M, N> >::getHandle();

			for (unsigned i = 0; i < M; i++)
				for (unsigned j = 0; j < N; j++)
					h->data(i, j) += rop.h->data(0, j);

			return static_cast< D<T, M, N> &> (*this);
		};

		/**
		 *	Swap rows of matrix
		 **/
		D<T, M, N>& row_swap( unsigned k, unsigned l)
		{
			if(h == NULL )
				return static_cast< D<T, M, N> &> (*this);

			RefHandle<DataContainer<T, M, N> >::getHandle();

			for (unsigned i = 0; i <  N; i++)
			{
				T temp=h->data(k,i);
				h->data(k,i)=h->data(l,i);
				h->data(l,i)=temp;

			}

			return static_cast< D<T, M, N> &> (*this);
		};


		/**
		 * In place subtract another matrix to this
		 **/
		D<T, M, N>& operator-=( BaseMatrix<D, T, M, N> const& rop)
		{
			return sub(rop);
		}
		D<T, M, N>& sub( BaseMatrix<D, T, M, N> const& __RESTRICTED rop)
		{
			if(h == NULL || rop.h == NULL)
				return static_cast< D<T, M, N> &> (*this);

			RefHandle<DataContainer<T, M, N> >::getHandle();

			for (unsigned i = 0; i < M; i++)
				for (unsigned j = 0; j < N; j++)
					h->data(i, j) -= rop.h->data(i, j);

			return static_cast< D<T, M, N> &> (*this);
		};


		/** Generic Multiply with another matrix
		 *	TODO: not faster than "slow" multiplication
		 **/
		template<unsigned L> D<T, M, L>  operator *( BaseMatrix<D, T, N, L> const& rop) const
		{
			return slow_mult(rop);
		}
		
		template<unsigned L> D<T, M, L>  slow_mult( BaseMatrix<D, T, N, L> const& __RESTRICTED rop) const
		{
			D<T, M, L> res;

			//std::cout<<"S";//<<std::endl;
			if(h == NULL || rop.h == NULL)
				return res;

			res.getHandle();

			//For each line of the resulting array
			for (unsigned i = 0; i < M; i++)
			{
				//For each element of this row
				for (unsigned j = 0; j < L; j++)
				{
					//Clear value
					res.h->data(i, j) = 0;
					T r = 0;

					for (unsigned k = 0; k < N; k++)
					{
						r += h->data(i, k) * rop.h->data(k, j);
					}

					res.h->data(i, j) = r;
				}
			}

			return res;
		};
		
		/** Generic Multiply with another square matrix, so result is same size as D<T,M,N>
		 *	TODO: not faster than "slow" multiplication
		 *	TODO: maybe "loose" the temp product space?
		 **/
		D<T, M, N> & operator *=( BaseMatrix<D, T, N, N> const& rop)
		{
			return fast_mult(rop);
		}
		
		D<T, M, N> & fast_mult( BaseMatrix<D, T, N, N> const&  rop) //in place mult!!!
		{
			if(h == NULL || rop.h == NULL)
				return static_cast< D<T, M, N> &> (*this);

			if(this->h == rop.h)
			{
				copyFrom(slow_mult(rop));
				return static_cast< D<T, M, N> &> (*this);
			}

			RefHandle<DataContainer<T, M, N> >::getHandle();
			T tmp[N];
			//std::cout<<"F";//<<h->counts<<std::endl;

			//For each line of the resulting array
			for (unsigned i = 0; i < M; i++)
			{
				//For each element of this row
				for (unsigned j = 0; j < N; j++)
				{
					//Clear value
					tmp[j] = 0;

					for (unsigned k = 0; k < N; k++)
					{
						tmp[j] += h->data(i, k) * rop.h->data(k, j);
					}
				}

				//std::cout<<"wtf:"<<std::endl;
				//prettyPrint();
				memcpy(h->data(i), tmp, sizeof(T[N]));
			}

			return static_cast< D<T, M, N> &> (*this);
		};

		//============================	Scalar operations ============================
		/**
		 * Add a scalar
		 */
		D<T, M, N>&  scalar_add(	const	 T	 scalar)
		{
			if(h == NULL)
				return static_cast< D<T, M, N> &> (*this);

			RefHandle<DataContainer<T, M, N> >::getHandle();

			for (unsigned i = 0; i < M; i++)
				for (unsigned j = 0; j < N; j++)
					h->data(i, j) += scalar;

			return static_cast< D<T, M, N>& > (*this);
		};

		T  sum() const
		{
			if(h == NULL)
				return 0;

			T s = 0;

			for (unsigned i = 0; i < M; i++)
				for (unsigned j = 0; j < N; j++)
					s += h->data(i, j);

			return s;
		};
		/**
		 * Subtract a scalar
		 */
		D<T, M, N>&  scalar_sub(const	T scalar)
		{
			if(h == NULL)
				return static_cast< D<T, M, N> &> (*this);

			RefHandle<DataContainer<T, M, N> >::getHandle();

			for (unsigned i = 0; i < M; i++)
				for (unsigned j = 0; j < N; j++)
					h->data(i, j) -= scalar;

			return static_cast< D<T, M, N>& > (*this);
		};
		/**
		 * Multiply with	a scalar
		 */
		D<T, M, N>& scalar_mult(const	T scalar)
		{
			if(h == NULL)
				return static_cast< D<T, M, N> &> (*this);

			RefHandle<DataContainer<T, M, N> >::getHandle();

			for (unsigned i = 0; i < M; i++)
				for (unsigned j = 0; j < N; j++)
					h->data(i, j) *= scalar;

			return static_cast< D<T, M, N> &> (*this);
		};

		/**
		 * Take the abs of the Matrix
		 */
		D<T, M, N> abs() const
		{
			D<T, M, N> ngen;

			if(h == NULL)
				return ngen;

			ngen.getHandle();

			for (unsigned i = 0; i < M; i++)
			{
				for (unsigned j = 0; j < N; j++)
				{
					ngen(j,i) = h->data(i, j)>0 ? h->data(i,j) : -h->data(i,j);
				}
			}

			return ngen;
		};

		/**
		 * Multiply with	a scalar
		 */
		D<T, M, N> operator* (const	T scalar)
		{
			D<T, M, N> res;
			res=(*this);
            res.scalar_mult(scalar);
            return res;
		};

		/**
		 * Transpose Matrix
		 */
		D<T, N, M> transp() const
		{
			D<T, N, M> ngen;
			
			if(h == NULL)
				return ngen;

			ngen.getHandle();

			for (unsigned i = 0; i < M; i++)
			{
				for (unsigned j = 0; j < N; j++)
				{
					ngen(j,i) =h->data(i, j);
				}
			}

			return ngen;
		};

		/**
		 * Return a new copy of this
		 **/
		D<T, M, N> clone() const
		{
			return D<T, M, N>(static_cast< D<T, M, N> const&> (*this));
			//return *(new D<T,M,N>(static_cast< D<T,M,N> const&> (*this))); //No biggie, COW
		};
		D<T, M, N> & copyTo(BaseMatrix<D, T, M, N> & dest ) const
		{
			dest = this;
			return static_cast< D<T, M, N> &> (*this);
		};
		D<T, M, N> & copyFrom(BaseMatrix<D, T, M, N> const & src )
		{
			RefHandle<DataContainer<T, M, N> >::operator=(src);
			//std::cout<<"sharing"<<h->counts<<std::endl;
			return static_cast< D<T, M, N> &> (*this);
		};

		/** Zero out matrix
		 *
		 **/
		D<T, M, N>& zero()
		{
			RefHandle<DataContainer<T, M, N> >::getHandle();
			h->zero();
			return static_cast< D<T, M, N> &> (*this);
		};

		/** Make matrix Identity (or in case its rectangular,pad with zeros)
		 *
		 **/
		D<T, M, N>& identity()
		{
			RefHandle<DataContainer<T, M, N> >::getHandle();
			h->zero();
			//Fill main diagonal
			unsigned l = M < N ? M : N;

			for (unsigned i = 0; i < l; i++)
				h->data(i, i) = 1;

			return static_cast< D<T, M, N> &> (*this);
		};
		//Accessor
		T& get(unsigned i, unsigned j)
		{
			RefHandle<DataContainer<T, M, N> >::getHandle();
#ifndef KMAT_INSANE_MODE

			if (i > M - 1 || j > N - 1)
			{
				std::string d("BaseMatrix.get() ");
				throw MatrixIndexOutOfBoundsException(d);
				//throw MatrixIndexOutOfBoundsException(d);
				return h->data(0, 0);
			}

#endif
			return h->data(i, j);
		};

		//Const accessor//Accessor
		const T read(unsigned i, unsigned j) const
		{
#ifndef KMAT_INSANE_MODE

			if(!RefHandle<DataContainer<T, M, N> >::validHandle())
				return 0;

			if (i > M - 1 || j > N - 1)
			{
				std::string d("BaseMatrix.get() ");
				throw MatrixIndexOutOfBoundsException(d);
				//throw MatrixIndexOutOfBoundsException(d);
				return h->data(0, 0);
			}

#endif
			return h->data(i, j);
		};

		/**
		 * For debuging mainly
		 */
		D<T, M, N> const& prettyPrint() const
		{
			using namespace std;
			cout << M << "x" << N << " Matrix" << endl;

			//Print header:
			if(h == NULL)
			{
				cout << "(Empty Matrix)" << endl;
				return static_cast< D<T, M, N> const&> (*this);
			}

			cout << "+";

			for (unsigned i = 0; i < N; i++)
				cout << "     -     ";

			cout << "+" << endl;

			for (unsigned i = 0; i < M; i++)
			{
				cout << "|";

				for (unsigned j = 0; j < N; j++)
				{
					//cout.width(7);
					//cout.precision(2);
					cout << setw(11) << setprecision(6) << fixed << h->data(i, j) << ""; //setprecision(3)<<setw(6)<<
				}

				cout << "|" << endl;
			}

			//Print footer:
			cout << "+";

			for (unsigned i = 0; i < N; i++)
				cout << "     -     ";

			cout << "+" << endl;
			return static_cast< D<T, M, N> const &> (*this);
		};
		//=== Operator overloading========
		COWRef<T, D<T, M, N> > operator() (unsigned i, unsigned j)
		{
			return COWRef<T, D<T, M, N> > ( static_cast< D<T, M, N>  &> (*this), i, j);
		};
		template<typename AT>D<T, M, N> &  operator= (BaseMatrix<D, AT, M, N> const & src)
		{
			for(unsigned i = 0; i < M; i++ )
				for(unsigned j = 0; j < N; j++)
					get(i, j) = src.read(i, j);

			return static_cast< D<T, M, N>  &> (*this);
		};
		//Const accessor
		const T operator() (unsigned i, unsigned j) const
		{
			return read(i, j);
		};
		/*D<T,M,N> & operator= (const D<T,M,N> & d)
		 {
		 return copyFrom(d);
		 };*/
		//
		T norm2() const
		{
			if(h == NULL)
				return 0;

			T res = 0, a;

			for(unsigned i = 0; i < M; i++)
				for(unsigned j = 0; j < N; j++)
				{
					a = h->data(i, j);
					res += a * a;
				}

			return res;
		}
		bool operator!=(BaseMatrix<D, T, M, N> const& other) const
		{
			for(unsigned i = 0; i < M; i++)
				for(unsigned j = 0; j < N; j++)
				{
					if(read(i, j) != other(i, j))
						return true;
				}

			return false;
		}
		bool operator==(BaseMatrix<D, T, M, N> const& other) const
		{
			return !(operator!=(other));
		}
	};
	//GenMatix: simply a BaseMatrix Instantation :)
	template <typename T, unsigned M, unsigned N>class GenMatrix: public BaseMatrix<GenMatrix, T, M, N>
	{
		public:
		GenMatrix<T, M, N> & pseudo_invert(GenMatrix<T, M, N> & athis);
		GenMatrix<T, M, N> & pseudoInverse()
		{
			if(this->h == NULL)
				return (*this);

			this->getHandle();
			return pseudo_invert(*this) ;
		};
	};
		
	//========================================Square matrices====================================
	//Transpose for square matrices
	template <typename A, unsigned S>
	GenMatrix<A, S, S> & transpose_square_matrix(GenMatrix<A, S, S> & athis)
	{
		for (unsigned i = 0; i < S; i++)
		{
			for (unsigned j = 0; j < i; j++)
			{
				A tempdata = athis.h->data[i][j];
				athis.h->data[i][j] = athis.h->data[j][i];
				athis.h->data[j][i] = tempdata;
			}
		}

		return athis;
	};
	//invert function declaration, used below as friend
	template <typename A, unsigned S>
	GenMatrix<A, S, S> & invert_square_matrix(GenMatrix<A, S, S> & athis);

	//Partial specialization for square matrices
	template<typename T, unsigned S> class GenMatrix<T, S, S> : public BaseMatrix<GenMatrix, T, S, S>
	{
		//using BaseMatrix<KMat:::GenMatrix,T,S,S>::data;
		friend GenMatrix<T, S, S> & transpose_square_matrix<>(GenMatrix<T, S, S> & athis);
		friend GenMatrix<T, S, S> & invert_square_matrix<>(GenMatrix<T, S, S> &athis);
	public:
		using BaseMatrix<KMat::GenMatrix, T, S, S>::operator=;
		GenMatrix<T, S, S> & fast_transp()
		{
			if(this->h == NULL)
				return (*this);

			this->getHandle();
			return transpose_square_matrix(*this) ;
		};//Override for square
		GenMatrix<T, S, S> & fast_invert()
		{
			if(this->h == NULL)
				return (*this);

			this->getHandle();
			return invert_square_matrix(*this) ;
		};//Override for square
	};

	template <typename T, unsigned S> class GenMatrix<T, S, 1> : public BaseMatrix<GenMatrix, T, S, 1>
	{
		//Add single dimentionall acess operator
	public:
		using BaseMatrix<KMat::GenMatrix , T, S, 1>::read;
		using BaseMatrix<KMat::GenMatrix , T, S, 1>::operator();
		using BaseMatrix<KMat::GenMatrix , T, S, 1>::operator=;

		COWRef<T, GenMatrix<T, S, 1> > operator() (unsigned i)
		{
			return COWRef<T, GenMatrix<T, S, 1> > ( static_cast< GenMatrix<T, S, 1>  &> (*this), i, 0);
		};
		const T operator() (unsigned i) const
		{
			return read(i, 0);
		};
	};




	template <typename T> class GenMatrix<T, 1, 1> : public BaseMatrix<GenMatrix, T, 1, 1>
	{
		//Add single dimentionall acess operator
	public:
		using BaseMatrix<KMat::GenMatrix , T, 1, 1>::read;
		using BaseMatrix<KMat::GenMatrix , T, 1, 1>::operator();
		using BaseMatrix<KMat::GenMatrix , T, 1, 1>::operator=;
		GenMatrix<T, 1, 1>() : BaseMatrix<KMat::GenMatrix, T, 1, 1>() {};
		explicit GenMatrix<T, 1, 1>(T ax) : BaseMatrix<KMat::GenMatrix, T, 1, 1>()
		{
			this->get(0, 0) = ax;
		};
		COWRef<T, GenMatrix<T, 1, 1> > operator() (unsigned i)
		{
			return COWRef<T, GenMatrix<T, 1, 1> > ( static_cast< GenMatrix<T, 1, 1>  &> (*this), i, 0);
		};
		const T operator() (unsigned i) const
		{
			return read(i, 0);
		};
		operator T () const{return read(0, 0);};

	};

	template <typename T> class GenMatrix<T, 2, 1> : public BaseMatrix<GenMatrix, T, 2, 1>
	{
		//Add single dimentionall acess operator
	public:
		using BaseMatrix<KMat::GenMatrix , T, 2, 1>::read;
		using BaseMatrix<KMat::GenMatrix , T, 2, 1>::operator();
		using BaseMatrix<KMat::GenMatrix , T, 2, 1>::operator=;
		GenMatrix<T, 2, 1>() : BaseMatrix<KMat::GenMatrix, T, 2, 1>() {};
		explicit GenMatrix<T, 2, 1>(T ax, T ay) : BaseMatrix<KMat::GenMatrix, T, 2, 1>()
		{
			this->get(0, 0) = ax;
			this->get(1, 0) = ay;
		};
		COWRef<T, GenMatrix<T, 2, 1> > operator() (unsigned i)
		{
			return COWRef<T, GenMatrix<T, 2, 1> > (   (*this), i, 0);
		};
		const T operator() (unsigned i) const
		{
			return read(i, 0);
		};
	};

	template <typename T> class GenMatrix<T, 3, 1> : public BaseMatrix<GenMatrix, T, 3, 1>
	{
		//Add single dimentionall acess operator
	public:
		using BaseMatrix<KMat::GenMatrix , T, 3, 1>::read;
		using BaseMatrix<KMat::GenMatrix , T, 3, 1>::operator();
		using BaseMatrix<KMat::GenMatrix , T, 3, 1>::operator=;
		GenMatrix<T, 3, 1>() : BaseMatrix<KMat::GenMatrix, T, 3, 1>() {};
		explicit GenMatrix<T, 3, 1>(T ax, T ay,T az) : BaseMatrix<KMat::GenMatrix, T, 3, 1>()
		{
			this->get(0, 0) = ax;
			this->get(1, 0) = ay;
			this->get(2, 0) = az;
		};
		COWRef<T, GenMatrix<T, 3, 1> > operator() (unsigned i)
		{
			return COWRef<T, GenMatrix<T, 3, 1> > (   (*this), i, 0);
		};
		const T operator() (unsigned i) const
		{
			return read(i, 0);
		};
	};




	// Matrix inversion	function, in-place gauss-jordan elimination with partial pivoting
	template <typename A, unsigned S>
	GenMatrix<A, S, S> & invert_square_matrix(GenMatrix<A, S, S> & athis)
	{

		GenMatrix<unsigned int,S,1>  ipivot;
		ipivot.zero();
		for(unsigned k=0;k<S;k++)
		{
			//Find pivot
			unsigned pvt=k;
			A pvtmx=(athis.read(k,k)>=0?athis.read(k,k):-athis.read(k,k));
			for (unsigned l=k+1;l<S;l++)
			{
				if(pvtmx<(athis(l,k)>=0?athis(l,k):-athis(l,k)))
				{
					pvt=l;
					pvtmx=(athis(l,k)>=0?athis(l,k):-athis(l,k));
				}
			}
			//Pivot
			//std::cout<<pvt<<":";
			//std::cout<<k<<std::endl;
			ipivot(k)=pvt;
			//ipivot.prettyPrint();
			athis.row_swap(k,pvt);

			if(athis(k,k)==0)
			{
				std::string d("KMat:invert_square_matrix<T,S,S>() ");
				throw SingularMatrixInvertionException(d);
			}

			//Eliminate all other rows
			for(unsigned j=0;j<S;j++)
			{
				if(j==k) continue;

				A m=-athis(j,k)/athis(k,k);

				for(unsigned q=0;q<S;q++)
				{
					athis(j,q)=athis(j,q)+m*athis(k,q);

				}
				athis(j,k)=m;

			}
			//Normalize current row
			A m_norm=1/athis(k,k);
			for(unsigned q=0;q<S;q++)
			{
				athis(k,q)=m_norm*athis(k,q);
			}
			athis(k,k)=m_norm;
		}
		//Repivot
		//ipivot.prettyPrint();
		for(int q=S-1;q>=0;q--)
		{
			athis.column_swap((unsigned)q, ipivot(q));
		}

		return athis;

	};
	template <typename A>
	GenMatrix<A, 2, 2> & invert_square_matrix(GenMatrix<A, 2, 2> & athis)
	{
		//using BaseMatrix<typename GenMatrix,2,2>::data;
		A determ = athis.read(0, 0) * athis.read(1, 1) - athis.read(0, 1) * athis.read(1, 0);

		//std::cout<<"Det:"<<determ<<std::endl;
		//std::cout<<"Eps:"<<std::numeric_limits<T>::epsilon()<<std::endl;
		if (determ > std::numeric_limits<A>::epsilon() && determ != (A)0) //can invert
		{
			A temp1 = athis.read(0, 0);
			athis.get(0, 0) = athis.read(1, 1) / determ;
			athis.get(1, 1) = temp1 / determ;
			A temp2 = athis.read(0, 1);
			athis.get(0, 1) = -athis.read(1, 0) / determ;
			athis.get(1, 0) = -temp2 / determ;
			return athis;
		}

		std::string d("KMat::invert_square_matrixGenMatrix<A,2,2>");
		throw SingularMatrixInvertionException(d);
	};

	template <typename A>
	GenMatrix<A, 3, 3> & invert_square_matrix(GenMatrix<A, 3, 3> & athis)
	{
		//using BaseMatrix<typename GenMatrix,2,2>::data;
		//Minor 1: based on 1,1
		A m1 = athis.read(0, 0) * (athis.read(1, 1) * athis.read(2, 2) - athis.read(1, 2) * athis.read(2, 1));
		//Minor 2: based on 1,2
		A m2 = athis.read(0, 1) * (athis.read(1, 0) * athis.read(2, 2) - athis.read(2, 0) * athis.read(1, 2));
		//Minor 2: based on 1,3
		A m3 = athis.read(0, 2) * (athis.read(1, 0) * athis.read(2, 1) - athis.read(2, 0) * athis.read(1, 1));
		A determ = m1 - m2 + m3;

		//std::cout<<"Det:"<<determ<<std::endl;
		//std::cout<<"Eps:"<<std::numeric_limits<A>::epsilon()<<std::endl;
		if (determ > std::numeric_limits<A>::epsilon() && determ != (A)0) //can invert
		{
			GenMatrix<A, 3, 3> t = athis.clone();
			athis.get(0, 0) = (t.read(1, 1) * t.read(2, 2) - t.read(1, 2) * t.read(2, 1)) / determ;
			athis.get(0, 1) = (t.read(0, 2) * t.read(2, 1) - t.read(0, 1) * t.read(2, 2)) / determ;
			athis.get(0, 2) = (t.read(0, 1) * t.read(1, 2) - t.read(0, 2) * t.read(1, 1)) / determ;
			athis.get(1, 0) = (t.read(1, 2) * t.read(2, 0) - t.read(1, 0) * t.read(2, 2)) / determ;
			athis.get(1, 1) = (t.read(0, 0) * t.read(2, 2) - t.read(0, 2) * t.read(2, 0)) / determ;
			athis.get(1, 2) = (t.read(0, 2) * t.read(1, 0) - t.read(0, 0) * t.read(1, 2)) / determ;
			athis.get(2, 0) = (t.read(1, 0) * t.read(2, 1) - t.read(1, 1) * t.read(2, 0)) / determ;
			athis.get(2, 1) = (t.read(0, 1) * t.read(2, 0) - t.read(0, 0) * t.read(2, 1)) / determ;
			athis.get(2, 2) = (t.read(0, 0) * t.read(1, 1) - t.read(0, 1) * t.read(1, 0)) / determ;
			return athis;
		}

		std::string d("KMat::invert_square_matrixGenMatrix<A,3,3>");
		throw SingularMatrixInvertionException(d);
	};
	//Pseudo Invertion of GenMatrix
	template <typename T, unsigned M, unsigned N>
	GenMatrix<T, M, N> & GenMatrix<T, M, N>::pseudo_invert(GenMatrix<T, M, N> & athis)
	{
		GenMatrix<T, M, N> ngen;
		GenMatrix<T, N, M> ngenTransp;
		GenMatrix<T, M, M> ngenInv;
		ngenTransp = athis.transp();
		ngenInv = athis*ngenTransp;
		//ngenInv.prettyPrint();
		ngenInv.fast_invert();
		ngen = ngenInv*athis;
		athis = ngen;
		return athis;
	};


	// Affine	transform matrix, using homogenous coordinates!!
	/* Internal represantation is one S-1 X S-1 matrix A
	 * and one S-1 x 1 B (homogenous coefs)
	 * | A B|
	 * | 0 1|
	 */
	class transformations;//Forward declaration
	template<typename T, unsigned S> class ATMatrix
	{

		friend class transformations;
	protected:
		GenMatrix < T, S - 1, S - 1 > A; //The main tranform matrix
		GenMatrix < T, S - 1, 1 > B; // The homogenous part of the transform
		/*
		 * Since this library is focused on rigid transformations, most of
		 * ATMatrices are going to be rigid transformations, so either B is
		 * going to be zero, or A is going to be identity...
		 * Therefore, keeping this extra info might just save a few cycles
		 * by avoiding unnecessary computations
		 */
		bool AisZero, AisIdentity, BisZero;
	public:
		ATMatrix(): AisZero(false), AisIdentity(false), BisZero(false) {};
		ATMatrix<T, S> & operator += ( ATMatrix<T, S> const& rop)
		{
			return add(rop);
		}
		ATMatrix<T, S> & add ( ATMatrix<T, S> const& rop)
		{
			A.add(rop.A);

			if (AisZero == true)
			{
				AisZero = rop.AisZero;
				AisIdentity = rop.AisIdentity;
			}
			else if (AisIdentity == true)
			{
				AisZero = false;
				AisIdentity = rop.AisZero; //If otherside is zero, remains Id
			}

			B.add(rop.B);

			if (BisZero == true)
				BisZero = rop.BisZero;

			return *this;
		};

		ATMatrix<T, S> & operator -= ( ATMatrix<T, S> const& rop)
		{
			return sub(rop);
		}

		ATMatrix<T, S> & sub ( ATMatrix<T, S> const& rop)
		{
			A.sub(rop.A);

			if (AisZero == true)
			{
				AisZero = rop.AisZero;
				AisIdentity = rop.AisIdentity;
			}
			else if (AisIdentity == true)
			{
				AisZero = false;
				AisIdentity = rop.AisZero; //If otherside is zero, remains Id
			}

			B.sub(rop.B);

			if (BisZero == true)
				BisZero = rop.BisZero;

			return *this;
		};
		ATMatrix<T, S>  operator* (ATMatrix<T, S> const& rop) const
		{
			return mult(rop);
		}

		ATMatrix<T, S>   mult (ATMatrix<T, S> const& rop) const
		{
			ATMatrix<T, S> res;
			res=(*this);
			res*=rop;
			return res;
		}

		//Multiplication, optimized for HTMatrices!!
		ATMatrix<T, S> & operator *= (ATMatrix<T, S> const& rop)
		{
			return fast_mult ( rop);
		}
		ATMatrix<T, S> & fast_mult (ATMatrix<T, S> const& rop)
		{
			GenMatrix < T, S - 1, 1 > axd = (A * rop.B);

			//std::cout<<"WTF!"<<std::endl;
			if (AisIdentity == true) //Mult with id , just copy
			{
				A = rop.A;
				AisZero = rop.AisZero;
				AisIdentity = rop.AisIdentity;
			}
			else if (AisZero == false) //Not id and not zero, just do the math
			{
				A *= rop.A;
				AisZero = rop.AisZero;
			}

			B += axd;

			if (BisZero == true)
				BisZero = rop.BisZero;

			return *this;
		}
		GenMatrix < T, S - 1, 1 > getTranslation() const
		{
			return B;
		}


		GenMatrix < T, S - 1, S - 1 > getRotation() const
		{
			return A;
		}

		ATMatrix<T, S> & setTranslation ( GenMatrix < T, S - 1, 1 > const& rop)
		{

			B=rop;
			BisZero=false;
			return *this;
		};
		GenMatrix < T, S - 1, 1 > getEulerAngles() const
		{
			GenMatrix < T, S - 1, 1 > r;
			r(0) = atan2(A(2, 1), A(2, 2));
			r(1) = atan2(-A(2, 0), sqrt(pow(A(2, 1), 2) + pow(A(2, 2), 2)));
			r(2) = atan2(A(1, 0), A(0, 0));
			return r;
		}
		//Project a point !
		template<unsigned M>
		GenMatrix < T, S - 1, 1 > transform(GenMatrix < T, S - 1, M > const & c, T hom = 1) const
		{
			GenMatrix < T, S - 1, M > nc;

			if (AisZero == true) //A matrix zero, dont try to do the math:p
			{
				nc.zero();//Result is definately zero
			}
			else if ( AisIdentity == true) //A matrix id, no change in c
			{
				nc = c; //Just clone coords
			}
			else//Just do the math :p
			{
				nc = A * c;
			}

			if (BisZero == false) //do some more math for the translatonal part
			{

				GenMatrix < T, S - 1, M > t = B;
				t.scalar_mult(hom);
				nc.column_add(t);
			}

			return nc;
		}

		ATMatrix<T, S> & fast_invert()
		{
			//It could be just the transpose, but only if A is a rotation... so there...
			if (AisIdentity != true)
				A.fast_invert();

			if (BisZero == false)
			{
				if (AisIdentity == false && AisZero == false) //Just do the math
				{
					GenMatrix < T, S - 1, 1 > newb = A * B;
					newb.scalar_mult(-1);
					B = newb;
				}
				else if (AisZero == true) //Result is def zero
					B.zero();
				else
				{
					B.scalar_mult(-1);
				}

				//else unchanged :) AisIdentity==true
			}

			return *this;
		}

		ATMatrix<T, S>& prettyPrint()
		{
			A.prettyPrint();
			B.prettyPrint();
			return *this;
		}

		bool almostEqualTo(ATMatrix<T, S> const & other, T tol=0.5)const
		{
			GenMatrix < T, S - 1, 1 > t,r;
			t=getTranslation();
			r=getEulerAngles();
			t-=other.getTranslation();
			r-=other.getEulerAngles();
			return sqrt(t.norm2())+sqrt(r.norm2())<tol;
		}

		ATMatrix<T, S>& identity()
		{
			A.identity();
			AisIdentity = true;
			AisZero = false;
			B.zero();
			BisZero = true;
			return *this;
		}
		ATMatrix<T, S>& zero()
		{
			A.zero();
			AisIdentity = false;
			AisZero = true;
			B.zero();
			BisZero = true;
			return *this;
		}
		ATMatrix<T, S>& check() //Just update booleans
		{
			BisZero = true;

			for (unsigned i = 0; i < S - 1; i++)
				if (B(i) != 0)
				{
					BisZero = false;
					break;
				}

			AisZero = true;
			AisIdentity = true;

			for (unsigned i = 0; i < S - 1; i++)
			{
				if (A(i, i) != 1)
					AisIdentity = false;

				for (unsigned j = 0; j < S - 1; j++)
				{
					if (A(i, j) != 0)
						AisZero = false;
				}

				if (AisIdentity == false && AisZero == false)
					break;
			}

			return *this;
		}
		COWRef<T, ATMatrix<T, S> > operator() (unsigned i, unsigned j)
		{
			AisIdentity=false;
			BisZero=false;
			return  COWRef<T, ATMatrix<T, S> >(*this, i, j);
		}

		const T operator() (unsigned i, unsigned j) const
		{
			return read(i, j);
		}

		T& get(unsigned i, unsigned j)
		{
#ifndef KMAT_INSANE_MODE

			if(i >= S - 1)
				throw MatrixIndexOutOfBoundsException("ATMatrix():");

#endif

			if(j == S - 1)
				return B.get(i, 0);
			else
				return A.get(i, j);
		}
		const T read(unsigned i, unsigned j) const
		{
#ifndef KMAT_INSANE_MODE

			if(i >= S - 1)
				throw MatrixIndexOutOfBoundsException("ATMatrix():");

#endif

			if(j == S - 1)
				return B.read(i, 0);
			else
				return A.read(i, j);
		};

	};
	//Typedef vector type
	template<typename T, unsigned S> struct Vector
	{

		typedef  GenMatrix<T, S, 1> type;

		static int isLeft(  typename KMat::Vector<T, 2>::type const& s,
		                    typename KMat::Vector<T, 2>::type const& e,
		                    typename KMat::Vector<T, 2>::type  const& t)
		{
			return (e.x - s.x) * (t.y - s.y) - (t.x - s.x) * (e.y - s.y);
		}
	};


	class transformations
	{
	public:
		static const double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811;
		template<typename T> static void makeRotation(ATMatrix<T, 3> & m, T theta)
		{
			m.identity();
			m.AisIdentity = false;
			makeRotation(m.A, theta);
		};
		template<typename T> static void makeRotation(GenMatrix<T, 2, 2> &m, T theta)
		{
			m(0, 0) = cos(theta);
			m(0, 1) = -sin(theta);
			m(1, 0) = sin(theta);
			m(1, 1) = cos(theta);
		};

		template<typename T> static void makeShearX(ATMatrix<T, 3> & m, T factor)
		{
			m.identity();
			m.AisIdentity = false;
			m.A(0, 1) = factor;
		};
		template<typename T> static void makeShearY(ATMatrix<T, 3> & m, T factor)
		{
			m.identity();
			m.AisIdentity = false;
			m.A(1, 0) = factor;
		};

		template<typename T, unsigned S> static void makeTranslation(ATMatrix<T, S> & m, GenMatrix < T, S - 1, 1 > const& t)
		{
			m.identity();
			m.AisIdentity = true;
			m.AisZero = false;
			m.BisZero = false;
			m.B.copyFrom(t);
		}
		template<typename T, unsigned S> static void makeScale(ATMatrix<T, S> &m, GenMatrix < T, S - 1, 1 > const & t)
		{
			m.identity();
			m.AisIdentity = false;

			for (unsigned i = 1; i < S - 1; i++)
				m.A(i, i) = t(i);
		}
		template<typename T> static void makeRotationZ(ATMatrix<T, 4> & m, T theta)
		{
			m.identity();
			m.AisIdentity = false;
			m.A(0, 0) = cos(theta);
			m.A(0, 1) = -sin(theta);
			m.A(1, 0) = sin(theta);
			m.A(1, 1) = cos(theta);
		}
		template<typename T> static void  makeRotationY(ATMatrix<T, 4> & m, T theta)
		{
			m.identity();
			m.AisIdentity = false;
			m.A(0, 0) = cos(theta);
			m.A(0, 2) = sin(theta);
			m.A(2, 0) = -sin(theta);
			m.A(2, 2) = cos(theta);
		}
		template<typename T> static void  makeRotationX(ATMatrix<T, 4> & m, T theta)
		{
			m.identity();
			m.AisIdentity = false;
			m.A(1, 1) = cos(theta);
			m.A(1, 2) = -sin(theta);
			m.A(2, 1) = sin(theta);
			m.A(2, 2) = cos(theta);
		}

		template<typename T> static void makeTranslation(ATMatrix<T, 4> & Transl, T x, T y, T z)
		{
			Transl.identity();
			Transl.AisIdentity = true;
			Transl.AisZero = false;
			Transl.BisZero = false;
			Transl(0, 3) = x;
			Transl(1, 3) = y;
			Transl(2, 3) = z;
		}
		/** Denavit Hartenberg transformation
		**/
		template<typename T> static void makeDHTransformation(ATMatrix<T, 4> & Transf, T a, T alpha, T d, T theta)
		{
			Transf.zero();
			Transf.AisZero = false;
			Transf.BisZero = false;
			Transf(0, 0) = cos(theta);
			Transf(0, 1) = -sin(theta);
			Transf(0, 2) = 0;
			Transf(0, 3) = a;
			Transf(1, 0) = sin(theta) * cos(alpha);
			Transf(1, 1) = cos(theta) * cos(alpha);
			Transf(1, 2) = -sin(alpha);
			Transf(1, 3) = -sin(alpha) * d;
			Transf(2, 0) = sin(theta) * sin(alpha);
			Transf(2, 1) = cos(theta) * sin(alpha);
			Transf(2, 2) = cos(alpha);
			Transf(2, 3) = cos(alpha) * d;
		}
		template<typename T> static void makeTransformation(ATMatrix<T, 4> & Transf, T px, T py, T pz, T rx, T ry, T rz)
		{
			Transf.zero();
			Transf.AisZero = false;
			Transf.BisZero = false;
			Transf(0, 0) = cos(rz) * cos(ry);
			Transf(0, 1) = cos(rz) * sin(ry) * sin(rx) - sin(rz) * cos(rx);
			Transf(0, 2) = cos(rz) * sin(ry) * cos(rx) + sin(rz) * sin(rx);
			Transf(0, 3) = px;
			Transf(1, 0) = sin(rz) * cos(ry);
			Transf(1, 1) = sin(rz) * sin(ry) * sin(rx) + cos(rz) * cos(rx);
			Transf(1, 2) = sin(rz) * sin(ry) * cos(rx) - cos(rz) * sin(rx);
			Transf(1, 3) = py;
			Transf(2, 0) = -sin(ry);
			Transf(2, 1) = cos(ry) * sin(rx);
			Transf(2, 2) = cos(ry) * cos(rx);
			Transf(2, 3) = pz;
		}
		template<typename T> static void makeRotationXYZ(ATMatrix<T, 4> & Rot, T xAngle, T yAngle, T zAngle)
		{
			ATMatrix<T, 4> Rx, Ry, Rz;
			makeRotationX(Rx, xAngle);
			makeRotationY(Ry, yAngle);
			makeRotationZ(Rz, zAngle);
			Rx *= Ry;
			Rx *= Rz;
			Rot = Rx;
		}
		template<typename T> static void makeRotationZYX(ATMatrix<T, 4> & Rot, T zAngle, T yAngle, T xAngle)
		{
			ATMatrix<T, 4> Rx, Ry, Rz;
			makeRotationX(Rx, xAngle);
			makeRotationY(Ry, yAngle);
			makeRotationZ(Rz, zAngle);
			Rz *= Ry;
			Rz *= Rx;
			Rot = Rz;
		}
		template<typename T> static GenMatrix<T, 4, 4> castToGenMatrix(ATMatrix<T, 4> At)
		{
			GenMatrix<T, 4, 4> Gen;
			Gen(0,0) = At(0,0);
			Gen(0,1) = At(0,1);
			Gen(0,2) = At(0,2);
			Gen(0,3) = At(0,3);
			Gen(1,0) = At(1,0);
			Gen(1,1) = At(1,1);
			Gen(1,2) = At(1,2);
			Gen(1,3) = At(1,3);
			Gen(2,0) = At(2,0);
			Gen(2,1) = At(2,1);
			Gen(2,2) = At(2,2);
			Gen(2,3) = At(2,3);
			Gen(3,0) = 0;
			Gen(3,1) = 0;
			Gen(3,2) = 0;
			Gen(3,3) = 1;
			return Gen;
		}
		
		template<typename T> static void makeDHDerivative(GenMatrix<T, 4, 4> & Gen, T alpha, T theta)
		{
			Gen.zero();
			Gen(0,0) = -sin(theta);
			Gen(0, 1) = -cos(theta);
			//Gen(0, 2) = 0;
			//Gen(0, 3) = 0;
			Gen(1, 0) = cos(theta) * cos(alpha);
			Gen(1, 1) = -sin(theta) * cos(alpha);
			//Gen(1, 2) = 0;
			//Gen(1, 3) = 0;
			Gen(2, 0) = cos(theta) * sin(alpha);
			Gen(2, 1) = -sin(theta) * sin(alpha);
			//Gen(2, 2) = 0;
			//Gen(2, 3) = 0;
			//Gen(3, 0) = 0;
			//Gen(3, 1) = 0;
			//Gen(3, 2) = 0;
			//Gen(3, 3) = 0;


			//kmatTable T2;
			//kmatJacobianTable T3;
			//KMatTransf::castToGenMatrix(T2, T3);
			//KMatTransf::makeDHTransformation(T2,0.0, -M_PI_2, 0.0, M_PI_2);
			//T3*=Transf;
			//T3.prettyPrint();
		}

	};

};

};

//Short Definitions :)
typedef KMath::KMat::Vector<double, 2>::type KVecDouble2;
typedef KMath::KMat::Vector<double, 3>::type KVecDouble3;
typedef KMath::KMat::Vector<float, 2>::type KVecFloat2;
typedef KMath::KMat::Vector<float, 3>::type KVecFloat3;
typedef KMath::KMat::Vector<int, 2>::type KVecInt2;
typedef KMath::KMat::Vector<int, 3>::type KVecInt3;


#endif
