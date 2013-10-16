package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;

/**
 * This class implements a structure to manage lists of 
 * (attribute,value) pairs (class AttributeValList).
 * In this class, the list is represented as a java.util.TreeSet, 
 * an ordered structure that optimizes the search of elements inside
 * it. So, in an AttributeValTreeSet, the AttributeVal objects can
 * be found in a efficient way.
 * <br>
 * @see CaseDescription
 * @see AttributeValList
 * @see java.util.TreeSet
 */
public class AttributeValTreeSet extends AttributeValList 
                                 implements java.io.Serializable
{

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

 
  /** the java TreeSet that stores the attributes */
    private java.util.TreeSet attrCase;
    
 

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  

  /**
   * Creates an empty AttributeValTreeSet, with the given attribute table and distance function. <br> 
   * <br>
   * {PRE: attrTab is instance of AttributeTable
   *       dist is an instance of ComposedDistance
   * }<br>
   * {POST: an instance of AttributeValTreeSet has been created.
   * }<br>
   */
    public AttributeValTreeSet(AttributeTable attrTab, ComposedDistance dist)
    {
       super(attrTab, dist);
       attrCase = new java.util.TreeSet();
    }
    
  
  //------------ mètodes abstractes de AttributeValList ----------

  /**
   * Adds the specified attribute to this AttributeValList, if it is not already present. <br>
   * {PRE: x is an instance of AttributeValTreeSet,
   *       o is an instance of a non-abstract subclass of AttributeVal 
   * }<br>
   * {POST: returns true if o was added correctly to x,
   *        false if there was a problem to add it
   * }<br>
   * @param   o attribute to be added to the v.
   * @return true if the AttributeValList is modified as a result
   *          of the invocation of this method.
   */
    public synchronized boolean add(AttributeVal a)
    {
       return attrCase.add(a);
    }
    

  /**
   * Adds all of the attributes in the specified AttributeValList to this 
   * AttributeValList.
   * {PRE: x is an instance of AttributeValTreeSet,
   *       c is an instance of a non-abstract subclass of AttributeValList 
   * }<br>
   * {POST: returns true if all the attributes in c were added correctly 
   *        to x, false if there were problems to add all of them.
   * }<br>
   * @param   c AttributeValList to be added to the AttributeValTreeSet.
   * @return true if the AttributeValTreeSet did not already contain any of the
   *          attributes in c.
   */
    public synchronized boolean addAll(AttributeValList c)
    {
       return attrCase.addAll(c);
    }
    

  /** 
   * Returns true if this AttributeValList contains the specified attribute. <br>
   * {PRE: x is an instance of AttributeValTreeSet,
   *       o is an instance of a non-abstract subclass of AttributeVal 
   * }<br>
   * {POST: returns true if o was found in x, 
   *        false otherwise
   * }<br>
   * @param   o attribute to be found in the AttributeValTreeSet.
   * @return true if the AttributeValTreeSet contains the specified attribute.
   */
    public boolean contains(AttributeVal o)
    {
       return attrCase.contains(o);
    }
    

  /** 
   * Returns true if this AttributeValList contains all of the attributes in the specified AttributeValList. <br>
   * {PRE: x is an instance of AttributeValTreeSet,
   *       c is an instance of a non-abstract subclass of AttributeValList 
   * }<br>
   * {POST: returns true if all the attributes of c were found at the AttributeValTreeSet,
   *        false if there isn't any of the attributes of c inside the AttributeValTreeSet
   * }<br>
   * @param   c collection to be checked at the AttributeValTreeSet.
   * @return true if the AttributeValTreeSet contains all the attributes in c.
   */
    public boolean containsAll(AttributeValList  c)
    {
       return attrCase.containsAll(c);
    }
    

  /**     
   * Removes the given attribute from this AttributeValList if it is present.
   * {PRE: x is an instance of AttributeValTreeSet,
   *       o is an instance of a non-abstract subclass of AttributeVal 
   * }<br>
   * {POST: returns true if o was found at the AttributeValTreeSet and so, removed.
   *        false if there isn't inside the AttributeValTreeSet.
   * }<br>
   * @param   o attribute to be removed at the AttributeValTreeSet.
   * @return true if the AttributeValTreeSet contained the specified attribute.
   */
    public synchronized boolean remove(AttributeVal o)
    {
       return attrCase.remove(o);
    }
    

  /**     
   * Removes the given attributes from this AttributeValTreeSet if they are present.
   * {PRE: x is an instance of AttributeValTreeSet,
   *       c is an instance of a non-abstract subclass of AttributeValList 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c AttributeValList to be removed from the AttributeValTreeSet.
   * @return true if the AttributeValTreeSet changed as a result of the call.
   */
    public synchronized boolean removeAll(AttributeValList  c)
    {
       return attrCase.removeAll(c);
    }
    

  /**
   * Retains only the attributes in this AttributeValTreeSet that are contained in the specified AttributeValList. <br>
   * That is, removes from the AttributeValTreeSet all of its attributes that are not 
   * contained in the specified AttributeValList. 
   * {PRE: x is an instance of AttributeValTreeSet,
   *       c is an instance of a non-abstract subclass of AttributeValList 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c AttributeValList to be retained at the AttributeValTreeSet, 
   *          removing the rest.
   * @return true if the AttributeValTreeSet changed as a result of the call.
   */   
     public synchronized boolean retainAll(AttributeValList  c)
    {
       return attrCase.retainAll(c);
    }

    
    //---- implementación de los metodos de Collection que no -----
    //---- implementa DistanceCollection ni CaseDescription ni ----
    //------------------    AttributeValList    -------------------
    
  /**
   * Returns an iterator over the elements (attributes) in this AttributeValTreeSet. <br>
   * The order of attributes returned is always the same.
   * <br>
   * {PRE: x is an instance of AttributeValTreeSet
   * }<br>
   * {POST: returns a java iterator to browse through the elements.
   * }<br>
   * @return Iterator a Java iterator for this structure
   */
    public java.util.Iterator iterator()
    {
       return attrCase.iterator();
    }
    
  /**
   * Returns the size of the AttributeValTreeSet structure. <br>
   * The size of the AttributeValTreeSet is the number of attributes stored.
   * If the AttributeValTreeSet contains more than
   * Integer.MAX_VALUE elements, returns Integer.MAX_VALUE.
   * <br>
   * {PRE: x is an instance of AttributeValTreeSet
   * }<br>
   * {POST: returns the number of attributes stored in x.
   * }<br>
   * @return int the number of attributes in the AttributeValTreeSet.
   */
    public int size()
    {
       return attrCase.size();
    }
    
  /**
   * Returns true if the AttributeValTreeSet structure contains no elements. <br>
   * <br>
   * {PRE: x is an instance of AttributeValTreeSet
   * }<br>
   * {POST: returns true if the the number of attributes stored in x is 0.
   * }<br>
   * @return boolean true if there are no attributes in the AttributeValTreeSet.
   */
    public boolean isEmpty()
    {
       return attrCase.isEmpty();
    }
    
  /**
   * Returns an array containing all of the elements in this collection in
   * a order. <br>
   * The returned array will be "safe" in that no references to it are
   * maintained by this collection. (In other words, this method must
   * allocate a new array even if this collection is backed by an array).
   * The caller is thus free to modify the returned array.
   * <br>
   * This method acts as bridge between array-based and collection-based
   * APIs.
   * {PRE: x is an instance of AttributeValTreeSet
   * }<br>
   * {POST: returns an array of objects containing a copy of all the 
            elements (attributes) in x.
   * }<br>
   * @return Object[] an array containing all the elements.
   */
    public Object[] toArray()
    {
       return attrCase.toArray();
    }
    
  /**
   * Returns an array containing all of the elements in this collection
   * whose runtime type is that of the specified array. If the collection
   * fits in the specified array, it is returned therein. Otherwise, a new
   * array is allocated with the runtime type of the specified array and
   * the size of this collection.
   * <br>
   * If this collection fits in the specified array with room to spare 
   * (i.e., the array has more elements than this collection), the element
   * in the array immediately following the end of the collection is set
   * to null. This is useful in determining the length of this collection
   * only if the caller knows that this collection does not contain any null
   * elements.)   
   * <br>
   * Like the toArray method, this method acts as bridge between 
   * array-based and collection-based APIs. Further, this method allows
   * precise control over the runtime type of the output array, and may,
   * under certain circumstances, be used to save allocation costs.
   * <br>
   * Note that toArray(new Object[0]) is identical in function to toArray().   
   * {PRE: x is an instance of AttributeValTreeSet,
   *       a is an empty array of objects 
   * }<br>
   * {POST: returns an array of all elements (attributes) in x which runtime
   *        type is the same of array a.
   * }<br>
   * @return Object[] an array containing some of the elements.
   */
    public Object[] toArray(Object[] a)
    {
       return attrCase.toArray(a);
    }  
    
  /**
   * Removes all of the elements from this collection. <br>
   * This collection will be empty after the call to the method.
   * {PRE: x is an instance of AttributeValTreeSet
   * }<br>
   * {POST: x is empty. 
   * }<br>
   */  
    public synchronized void clear()
    {
       attrCase.clear();
    }
    

  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of AttributeValTreeSet
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public String toString()
    {
       return (attrCase.toString());
    }
}