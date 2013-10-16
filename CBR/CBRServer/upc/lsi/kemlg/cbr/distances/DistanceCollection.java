package upc.lsi.kemlg.cbr.distances;

/**
 * This interface defines the methods that should have any collection
 * that manages objects where distance functions can be applied
 * (that is, objects following the DistanceComparable interface).
 * <br>
 * @see Collection
 * @see DistanceComparable
 */
public interface DistanceCollection extends java.util.Collection,
                                            java.io.Serializable
{   

  //====||===============================================================
  //===\||/======= metodos a implementar por las clases =================
  //====\/===============================================================
  
  /**
   * Computes the distance among this DistanceCollection and a given one. <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of DistanceCollection,
   *       d2 is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: returns the computed distance among x and d2.
   * }<br>
   * @param   d2 the DistanceCollection to be considered.
   * @return float the distance among x and d2.
   */
    public float distanceTo(DistanceCollection d2);

    
  /**
   * Gets the current distance function that applies to all the elements of this DistanceCollection. <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of DistanceCollection,
   * }<br>
   * {POST: returns the distance function that is applied for all the elements.
   * }<br>
   * @return ComposedDistance the distance function.
   */
    public ComposedDistance getCurrentDistance();

    
  /**
   * Adds the specified element to this DistanceCollection, if it is not already present. <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of DistanceCollection,
   *       o is an instance of a non-abstract subclass of DistanceComparable 
   * }<br>
   * {POST: returns true if o was added correctly to x,
   *        false if there was a problem to add it
   * }<br>
   * @param   o element to be added to the DistanceCollection.
   * @return true if the DistanceCollection is modified as a result
   *          of the invocation of this method.
   */
    public abstract boolean add(DistanceComparable o);
    

  /**
   * Adds all of the elements in the specified collection to this 
   * DistanceCollection.
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of DistanceCollection,
   *       c is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: returns true if all the elements in c were added correctly 
   *        to x, false if there were problems to add all of them.
   * }<br>
   * @param   c collection to be added to the DistanceCollection.
   * @return true if the DistanceCollection did not already contain any of the
   *          elements in c.
   */
    public abstract boolean addAll(DistanceCollection c);
    

  /** 
   * Returns true if this DistanceCollection contains the specified element. <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of DistanceCollection,
   *       o is an instance of a non-abstract subclass of DistanceComparable 
   * }<br>
   * {POST: returns true if o was found in x, 
   *        false otherwise
   * }<br>
   * @param   o element to be found in the DistanceCollection.
   * @return true if the DistanceCollection contains the specified element.
   */
    public abstract boolean contains(DistanceComparable o);


  /** 
   * Returns true if this DistanceCollection contains all of the elements in the specified collection. <br>
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of DistanceCollection,
   *       c is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: returns true if all the elements of c were found at the DistanceCollection,
   *        false if there isn't any of the elements of c inside the DistanceCollection
   * }<br>
   * @param   c collection to be checked at the DistanceCollection.
   * @return true if the DistanceCollection contains all the elements in c.
   */
    public abstract boolean containsAll(DistanceCollection c);
    

  /**     
   * Removes the given element from this DistanceCollection if it is present.
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of DistanceCollection,
   *       o is an instance of a non-abstract subclass of DistanceComparable 
   * }<br>
   * {POST: returns true if o was found at the DistanceCollection and so, removed.
   *        false if there isn't inside the DistanceCollection.
   * }<br>
   * @param   o element to be removed at the DistanceCollection.
   * @return true if the DistanceCollection contained the specified element.
   */
    public abstract boolean remove(DistanceComparable o);


  /**     
   * Removes the given elements from this DistanceCollection if they are present.
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of DistanceCollection,
   *       c is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c collection of elements to be removed from the DistanceCollection.
   * @return true if the DistanceCollection changed as a result of the call.
   */
    public abstract boolean removeAll(DistanceCollection c);
    

  /**
   * Retains only the elements in this DistanceCollection that are contained in the specified collection. <br>
   * That is, removes from the DistanceCollection all of its elements that are not 
   * contained in the specified collection. 
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of DistanceCollection,
   *       c is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c collection of elements to be retained at the DistanceCollection, 
   *          removing the rest.
   * @return true if the DistanceCollection changed as a result of the call.
   */   
    public abstract boolean retainAll(DistanceCollection c);
    
}