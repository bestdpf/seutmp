/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef TREE_NODE_HPP
#define TREE_NODE_HPP

#include <list>
#include <string>
#include <boost/weak_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <ostream>
#include "ClassException.hpp"
#include "Template.hpp"
#include <iostream>

namespace tree
{
    using namespace std;
    using namespace boost;
    
#if defined(_MSC_VER)
	// VC2005 warning
	#pragma warning(disable:4996)
	// VC exception warning
	#pragma warning(disable:4290)
#endif


    struct LIST_TREE{};         /**< the children are stored in a list */
    struct BINARY_TREE{};       /**< the children are stored in a binary tree */
    
    /**
     * the node class to creat a tree struct storage, and with some
     * operations of tree. It is implemented in node - sub-nodes way
     */

    /* declare the base node */
    template <typename CHILDTYPE, typename IMPLEMENT>
    class BaseNode;

    /* LIST implement */
    template <typename CHILDTYPE> 
    class BaseNode<CHILDTYPE, LIST_TREE>
    {
    public:
        /// the list stores children
        typedef list< shared_ptr<CHILDTYPE> > TChildList;

        /// the destruction
        ~BaseNode(){}

        /** 
         * create a child
         * 
         * @return a smart pointer pointing to the created child
         */
        static shared_ptr<CHILDTYPE> create()
            {
                shared_ptr<CHILDTYPE> p(new CHILDTYPE );
                p->setSelf( p );
                return p;
            }

        /** 
         * add a node as child to this node
         * 
         * @param child the smart point to the child node
         * 
         * @return if the operation is successful
         */
        bool addChild( shared_ptr<CHILDTYPE> child)
            {
                if ( NULL==child.get() ) return false;
                mChildren.push_back(child);
                child->setParent( mSelf );
                return true;
            }

        /** 
         * get the children of this node
         * 
         * @return the list of children
         */
        TChildList& children()
            {
                return mChildren;
            }

        /** 
         * get the children of this node
         * 
         * @return the constant list of children
         */
        const TChildList& children() const
            {
                return mChildren;
            }

        /** 
         * get the parent node of this node
         * 
         * @return the weak smart point to parent node
         */
        shared_ptr<CHILDTYPE> parent()
            {
                return mParent.lock();//make_shared(mParent);
            }

        /** 
         * get the parent node of this node
         * 
         * @return the weak smart point to parent node ( constancy )
         */
        shared_ptr<const CHILDTYPE> parent() const
            {
                return mParent.lock();//make_shared(mParent);
            }

        /** 
         * get the pointer of this node
         * 
         * @return the weak smart point to this node
         */
        shared_ptr<CHILDTYPE> self()
            {
                return mSelf.lock();//make_shared(mSelf);
            }

        /** 
         * get the pointer of this node
         * 
         * @return the weak smart point to this node ( constancy )
         */
        shared_ptr<const CHILDTYPE> self() const
            {
                return mSelf.lock();//make_shared(mSelf);
            }
        
        /** 
         * get the root of this tree, i.e. the top node without parent
         * 
         * @return the root of this tree
         */
        shared_ptr<CHILDTYPE> root()
            {
                shared_ptr<CHILDTYPE> node = parent();
                if ( NULL == node.get() )
                    return self();
                else
                    return node->root();
            }

        shared_ptr<const CHILDTYPE> root() const
            {
                shared_ptr<const CHILDTYPE> node = parent();
                if ( NULL == node.get() )
                    return self();
                else
                    return node->root();
            }

        shared_ptr<CHILDTYPE> sister()
            {
                shared_ptr<CHILDTYPE> p = parent();
                if ( NULL != p.get() ){
                    FOR_EACH( iter, p->children() ){
                        if ( this == (*iter).get() ){
                            if ( p->children().end() != ++iter ){
                                return *iter;
                            }
                            return shared_ptr<CHILDTYPE>();
                        }
                    }
                }
                return shared_ptr<CHILDTYPE>();
            }

        shared_ptr<const CHILDTYPE> sister() const
            {
                shared_ptr<const CHILDTYPE> p = parent();
                if ( NULL != p.get() ){
                    FOR_EACH( iter, p->children() ){
                        if ( this == (*iter).get() ){
                            if ( p->children().end() != ++iter ){
                                return *iter;
                            }
                            return shared_ptr<const CHILDTYPE>();
                        }
                    }
                }
                return shared_ptr<const CHILDTYPE>();
            }

        shared_ptr<CHILDTYPE> child()
            {
                if ( mChildren.empty() )
                    return shared_ptr<CHILDTYPE>();
                return *(mChildren.begin());
            }

        shared_ptr<const CHILDTYPE> child() const
            {
                if ( mChildren.empty() )
                    return shared_ptr<const CHILDTYPE>();
                return *(mChildren.begin());
            }
        
    protected:

        /// do not call the construction directly
        BaseNode(){}
        

    private:
        /** 
         * set the weak smart point to this node self, this point is used
         * when linking to another node
         * 
         * @param self the smart point to this node
         */
        void setSelf( shared_ptr<CHILDTYPE> self ) { mSelf = self; }

        /** 
         * set the parent node of this node
         * 
         * @param parent the weak smart point to parent node
         */
        void setParent( weak_ptr<CHILDTYPE> parent ){ mParent = parent; }

        /** 
         * break the link from parent node, i.e. destroy the weak smart
         * point to the parent
         */
        void unLink(){ mParent.reset(); }
        
        /// list of children
        TChildList mChildren;

        /// weak smart pointer to parent
        weak_ptr<CHILDTYPE> mParent;

        /// weak smart pointer to self
        weak_ptr<CHILDTYPE> mSelf;
    };

    /* binary implement */
    template <typename CHILDTYPE> 
    class BaseNode<CHILDTYPE, BINARY_TREE>
    {
    public:
        /// the destruction
        ~BaseNode(){}

        /** 
         * create a child
         * 
         * @return a smart pointer pointing to the created child
         */
        static shared_ptr<CHILDTYPE> create()
            {
                shared_ptr<CHILDTYPE> p(new CHILDTYPE );
                p->setSelf( p );
                return p;
            }

        /** 
         * add a node as child to this node
         * 
         * @param child the smart point to the child node
         * 
         * @return if the operation is successful
         */
        bool addChild( shared_ptr<CHILDTYPE> child)
            {
                if ( NULL==child.get() ) return false;
                if ( NULL == mChild.get() ){
                    mChild = child;
                    child->setParent( mSelf );
                    return true;
                }
                else {
                    return mChild->addSister(child);
                }
            }

        bool addSister( shared_ptr<CHILDTYPE> sis )
            {
                if ( NULL==sis.get() ) return false;
                if ( NULL==mSister.get() ){
                    mSister = sis;
                    sis->setParent( parent() );
                    return true;
                }
                else{
                    return mSister->addSister(sis);
                }
            }

        /** 
         * get the children of this node
         * 
         * @return the list of children
         */
        list< shared_ptr<CHILDTYPE> > children()
            {
                list< shared_ptr<CHILDTYPE> > chs;
                shared_ptr<CHILDTYPE> ch = child();
                while ( NULL != ch.get() ){
                    chs.push_back(ch);
                    ch = ch->sister();
                }
                return chs;
            }

        list< shared_ptr<const CHILDTYPE> > children() const
            {
                list< shared_ptr<const CHILDTYPE> > chs;
                shared_ptr<const CHILDTYPE> ch = child();
                while ( NULL != ch.get() ){
                    chs.push_back(ch);
                    ch = ch->sister();
                }
                return chs;
            }

        /** 
         * get the parent node of this node
         * 
         * @return the weak smart point to parent node
         */
        shared_ptr<CHILDTYPE> parent()
            {
                return mParent.lock();//make_shared(mParent);
            }

        /** 
         * get the parent node of this node
         * 
         * @return the weak smart point to parent node ( constancy )
         */
        shared_ptr<const CHILDTYPE> parent() const
            {
                return mParent.lock();//make_shared(mParent);
            }

        /** 
         * get the pointer of this node
         * 
         * @return the weak smart point to this node
         */
        shared_ptr<CHILDTYPE> self()
            {
                return mSelf.lock();//make_shared(mSelf);
            }

        /** 
         * get the pointer of this node
         * 
         * @return the weak smart point to this node ( constancy )
         */
        shared_ptr<const CHILDTYPE> self() const
            {
                return mSelf.lock();//make_shared(mSelf);
            }
        
        /** 
         * get the root of this tree, i.e. the top node without parent
         * 
         * @return the root of this tree
         */
        shared_ptr<CHILDTYPE> root()
            {
                shared_ptr<CHILDTYPE> node = parent();
                if ( NULL == node.get() )
                    return self();
                else
                    return node->root();
            }

        shared_ptr<const CHILDTYPE> root() const
            {
                shared_ptr<const CHILDTYPE> node = parent();
                if ( NULL == node.get() )
                    return self();
                else
                    return node->root();
            }

        shared_ptr<CHILDTYPE> sister()
            {
                return mSister;
            }

        shared_ptr<const CHILDTYPE> sister() const
            {
                return mSister;
            }

        shared_ptr<CHILDTYPE> child()
            {
                return mChild;
            }

        shared_ptr<const CHILDTYPE> child() const
            {
                return mChild;
            }
        
    protected:

        /// do not call the construction directly
        BaseNode(){}
        

    private:
        /** 
         * set the weak smart point to this node self, this point is used
         * when linking to another node
         * 
         * @param self the smart point to this node
         */
        void setSelf( shared_ptr<CHILDTYPE> self ) { mSelf = self; }

        /** 
         * set the parent node of this node
         * 
         * @param parent the weak smart point to parent node
         */
        void setParent( weak_ptr<CHILDTYPE> parent ){ mParent = parent; }

        /** 
         * break the link from parent node, i.e. destroy the weak smart
         * point to the parent
         */
        void unLink(){ mParent.reset(); }

        /// the sister
        shared_ptr<CHILDTYPE> mSister;
        
        /// the child
        shared_ptr<CHILDTYPE> mChild;

        /// weak smart pointer to parent
        weak_ptr<CHILDTYPE> mParent;

        /// weak smart pointer to self
        weak_ptr<CHILDTYPE> mSelf;
    };
    
    template <class DATATYPE, typename IMPLEMENT=LIST_TREE>
    class Node : public BaseNode< Node<DATATYPE>, IMPLEMENT>
    {
    public:
        /** 
         * make the constructor private
         * 
         * @param data the smart point to data
         */
        Node(){}
        
        /** 
         * the destruct function
         * 
         */
        ~Node()
            {
                //std::cout<<__FUNCTION__<<'\n';
            }
        
        /** 
         * create function to new a node
         * 
         * @param data the smart point to data
         * 
         * @return the smart point to node which holds the data
         */
        static shared_ptr<Node> create( shared_ptr<DATATYPE> data )
            {
                shared_ptr<Node> p = BaseNode<Node,IMPLEMENT>::create();
                p->data() = data;
                return p;
            }

        /** 
         * new a smart point to node, and copy the data from source node
         * 
         * @param o the source node
         * @param copy the copy function of DATATYPE
         * 
         * @return the smart point to new node
         */
        static shared_ptr<Node> create(
            shared_ptr<const Node> o,
            typename std::const_mem_fun_t<shared_ptr<DATATYPE>,const DATATYPE> copy )
            {
                const DATATYPE *dataPtr = o->data().get();
                if( 0 != dataPtr )
                {
                    // the class DATATYPE should define the copy func
                    shared_ptr<DATATYPE> data = copy( dataPtr );
                    return create(data);
                }
                return shared_ptr<Node>();
            }

        /** 
         * new a node to hold data, and add it to another node as parent
         * to build a tree
         * 
         * @param data the smart point to data
         * @param parent the smart point to node, which will be parent
         * 
         * @return the smart point to new node
         */
        static shared_ptr<Node> create(shared_ptr<DATATYPE> data,
                                       shared_ptr<Node> parent )
            {
                shared_ptr<Node> p = create(data);
                parent->addChild( p );
                return p;
            }

        /** 
         * remove a child node of this node
         * 
         * @param name the child node's name in string
         * 
         * @return if the operation is successful
         */
        // bool removeChild(const string& name)
        //     {
        //         for( typename TNodeList::iterator iter = mChildren.begin();
        //              iter != mChildren.end();
        //              ++iter )
        //         {
        //             if ( name == (*iter)->getName() )
        //             {
        //                 iter->unLink();
        //                 mChildren.erase(iter);
        //                 return true;
        //             }
        //         }
        //         return false;
        //     }

        /** 
         * get the child by a given name
         * 
         * @param name the child node's name in string
         * 
         * @return the smart point to child node which has the same name
         * as given
         */
        shared_ptr<Node> getChild(const string& name)
            {
                FOR_EACH( iter, this->children() )
                {
                    if ( name == (*iter)->data()->getName() )
                    {
                        return *iter;
                    }
                }
                return shared_ptr<Node>();
            }

        /** 
         * get the name of this node, the node do not have a member as its
         * name, so the data should take response to it.
         * 
         * @return the name of this node
         */
        const string& getName() const
            {
                return data()->getName();
            }

        /** 
         * get the data of this node
         * 
         * @return the smart point to data
         */
        shared_ptr<DATATYPE>& data()
            {
                return mData;
            }

        /** 
         * get the data of this node
         * 
         * @return the smart point to data ( constancy )
         */
        shared_ptr<const DATATYPE> data() const
            {
                return mData;
            }

        /** 
         * copy the nodes from a given node recursively
         * 
         * @param o the source node which will be copied
         */
        void copy(shared_ptr<const Node> o)
            {
                FOR_EACH(iter, o->children())
                {
                    shared_ptr<Node> child = create( (*iter)->data() );
                    addChild(child);
                    // call recursively
                    child->copy(*iter);
                }
            }

        /** 
         * copy the child nodes from a given node
         * 
         * @param o the source node whose child nodes will be copied to
         * this node
         * @param copy the copy function of data type
         */
        void deepCopy(
            shared_ptr<const Node> o,
            typename std::const_mem_fun_t<shared_ptr<DATATYPE>,DATATYPE> copy)
            {
                FOR_EACH(iter, o->children()){
                    shared_ptr<Node> child = create(*iter,copy);
                    addChild(child);
                    // call recursively
                    child->copyChildren(*iter,copy);
                }
            }

        /** 
         * do something ( call function ) from top(this node) to down(leaf
         * node) in the tree recursively
         * 
         * @param func the function which will be called by every child
         * node
         */
        template <typename DATATYPE_FUNC>
        void topDownRecursivelyDo( DATATYPE_FUNC func ) const
            {
                const DATATYPE* ptr = data().get();
                if ( 0!=ptr ) func( ptr );

                FOR_EACH( iter, this->children() )
                {
                    (*iter)->topDownRecursivelyDo( func );
                }
            }

        /** 
         * do something ( call function ) from top(this node) to down(leaf
         * node) in the tree recursively, in such a way as push and pop
         * 
         * @param funcBegin the function will be called firstly
         * @param funcEnd the function will be called lastly
         */
        template <typename DATATYPE_BEGIN_FUNC, typename DATATYPE_END_FUNC>
        void topDownRecursivelyDo( DATATYPE_BEGIN_FUNC funcBegin,
                                   DATATYPE_END_FUNC funcEnd) const
            {
                const DATATYPE* ptr = data().get();
                if ( 0!=ptr ) funcBegin( ptr );

                FOR_EACH( iter, this->children() )
                {
                    (*iter)->topDownRecursivelyDo(funcBegin,funcEnd);
                }
		
                if ( 0!=ptr ) funcEnd( ptr );
            }

        /** 
         * do something ( call function ) from down(this node) to
         * top(root) in the tree recursively
         * 
         * @param func the function will be called by nodes
         */
        template <typename DATATYPE_FUNC>
        void bottomUpRecursivelyDo( DATATYPE_FUNC func )
            {
                DATATYPE* ptr = data().get();
                if ( 0!=ptr ) func( ptr );
                
                shared_ptr< Node > p = (this->parent()).lock();//make_shared( this->parent() );
                if( 0 != p.get() )
                {
                    p->bottomUpRecursivelyDo( func );
                }
            }

        /** 
         * do something ( call function ) from down(this node) to
         * top(root) in the tree recursively, but the function is
         * constancy
         * 
         * @param func the constancy function will be called by nodes
         */
        template <typename DATATYPE_FUNC>
        void bottomUpRecursivelyDo( DATATYPE_FUNC  func ) const
            {
                const DATATYPE* ptr = data().get();
                if ( 0!=ptr ) func( ptr );
                
                shared_ptr<const Node > p = this->parent();
                if( 0 != p.get() )
                {
                    p->bottomUpRecursivelyDo( func );
                }
            }

        /** 
         * find out the node according the given path
         * 
         * @param path the path from this node to the target node
         * 
         * @return the target node
         */
        shared_ptr<Node> findByPath(string path)
            {
                shared_ptr<Node> node = this->self();
                if ( '/' == path[0] ){
                    node = node->root();
                    path.erase(0,1);
                }

                size_t i = path.find("/");
                string tmp;
                while( i!=string::npos ){
                    tmp.assign(path,0,i);
                    if ( ".." == tmp ){
                        node = node->parent();
                    }
                    else{
                        node = node->getChild(tmp);
                    }
                    path.erase(0,i+1);
                    i = path.find("/");
                }
                if ( !path.empty())
                    node = node->getChild(path);
            
                return node;
            }


        /** 
         * list all devices of DEVICE in the node
         * 
         * @param root the input root node
         * @param results the results
         */
        template <typename T>
        void listChildren(list< shared_ptr<const Node> >& results, bool recursively = false ) const
            {
                FOR_EACH( iter, this->children() ){
            shared_ptr< const Node > node = *iter;
            // try to cast the pointer
            shared_ptr<const T> dev =
                shared_dynamic_cast<const T>(node->data());
            // if casted successfully, add it to the list
            if ( 0 != dev.get() )
                results.push_back(node);
            // call nested
            if ( recursively ){
                node->listChildren<T>(results);
            }
        }}

        template <typename T>
        void listChildren(list< shared_ptr<Node> >& results, bool recursively = false )
        {
            FOR_EACH( iter, this->children() ){
            shared_ptr< Node > node = *iter;
            // try to cast the pointer
            shared_ptr<T> dev =
                shared_dynamic_cast<T>(node->data());
            // if casted successfully, add it to the list
            if ( 0 != dev.get() )
                results.push_back(node);
            // call nested
            if ( recursively ){
                node->listChildren<T>(results);
            }
        }}


        template <typename T>
        shared_ptr<const Node> getTheOneChild(bool recursively = false ) const
        {
            list< shared_ptr<const Node> > devList;
            listChildren<T>(devList, recursively);
            if ( devList.size() > 1 ){
                cerr<<__FUNCTION__<<' '<<getName()<<" has "
                    <<devList.size()<<" "<<typeid(T).name()<<endl;
            }
            if ( devList.empty() )
                return shared_ptr<const Node>();
            
            return devList.front();
        }

        template <typename T>
        shared_ptr<Node> getTheOneChild(bool recursively = false )
        {
            list< shared_ptr<Node> > devList;
            listChildren<T>(devList, recursively);
            if ( devList.size() > 1 ){
                cerr<<__FUNCTION__<<' '<<getName()<<" has "
                    <<devList.size()<<" "<<typeid(T).name()<<endl;
            }
            if ( devList.empty() )
                return shared_ptr<Node>();
            
            return devList.front();
        }

    private:
        
        /// holds data
        shared_ptr<DATATYPE> mData;
    };

    /** 
     * the dump function of a tree
     * 
     * @param os the output stream
     * @param node the root node of a tree
     * 
     * @return the output stream
     */
    template <class DATATYPE>
    ostream& operator<<(ostream& os, const Node<DATATYPE>& node)
    {
        static int count = 0;
        os<<node.getName();
        if ( node.children().empty() )
        {
            os<<'\n';
            return os;
        }
        count++;
        os<<"\t- ";
        FOR_EACH( iter, node.children() )
        {
            if ( iter != node.children().begin() )
            {
                for( int i=count; i>0; i-- )
                {
                    os<<"\t";
                }
                os<<"+ ";
            }
            os<<(*(*iter));
        }
        count--;
        return os;
    }

} // namespace tree

#endif // TREE_NODE_HPP
