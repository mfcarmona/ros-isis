package upc.lsi.kemlg.general;

import java.io.*;
import java.util.LinkedList;
import java.util.Iterator;


/**
 * This class keps an ordered list of the N higher values added to it.<br>
 * The TopNList class implements the java List interface, and provides an 
 * ordered list of elements with a maximum size N. It only keeps the N higher
 * values added to the list (that is, the "top N values"), adn drops some of
 * the lower ones if there are more than N values in the list. The default 
 * maximum size is 100, but it can be set by one of the constructors of the 
 * class.<br> 
 * The ordering of the list is the order defined by its components, which 
 * should implement the Comparable interface.<br>
 * <b>Note that this implementation is not synchronized</b>. If multiple threads
 * access a list concurrently, and at least one of the threads modifies the
 * list structurally, it must be synchronized externally. 
 */
public class TopNList extends LinkedList implements Serializable 
{
  
  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================
  
  /**
   * The maximum number of elements stored (the maximum size of the list).
   */
   private int MAXSIZE = 100;
            

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================

  /**
   * Creates an empty TopNList.<br>
   * The maximum size is the default one, 100 elements.<br>
   * {PRE: true
   * }<br>
   * {POST: an empty TopNList is created, with the default maximum size.
   * }<br>
   */
   public TopNList()
   {
   	 super();
   }
   
   
  /**
   * Creates an empty TopNList with a maximum size of maxsize.<br>
   * {PRE: maxsize is a positive integer
   * }<br>
   * {POST: an empty TopNList is created, with the maximum size maxsize.
   * }<br>
   * @param maxsize the maximum size of the list.
   */
   public TopNList(int maxsize)
   {
     super();
     MAXSIZE = maxsize;
   }
   

  /**
   * Adds the object o to the list if it's one of the N higher values
   * (N=MAXSIZE), and drops the lower value if there are more than MAXSIZE
   * values stored.
   * {PRE: x is a TopNList, and o is a Object comparable to the ones already
   *       inside of x.
   * }<br>
   * {POST: x contains the N higher elements of the set x + {o}
   * }<br>
   * @return boolean x changes as a result of the method call.
   */ 
	 public boolean add (Object o)
   {
      Object o_act;
      boolean encontrada;
      int pos;
    
      if (!(o instanceof Comparable))
      {
         throw new ClassCastException("Cannot add this element to a TopNList. It should implement the interface Comparable");
      }
      else
      {  
      
         //Buscar la posición en la que hay que insertar
         pos = 0;
         encontrada = false;
         for (Iterator i = this.iterator(); (i.hasNext() && !encontrada); pos++)
         {
            o_act = i.next();
            encontrada = ((Comparable) o).compareTo(o_act) >= 0;
         }
      
         //añadir el elemento al vector  
         if (encontrada)
         { 
            //añadirlo a la posición
            add(pos-1,o);
         
            //eliminar un elemento al final, si es necesario  
            if (this.size()>MAXSIZE)
            {
               this.removeLast();
            }           
         }
         else
         {
            // se añade al final... excepto si queda fuera de los N
            // mejores.
            if(this.size()<MAXSIZE)
            {  
               super.add(o);
            }
         }
      }
       
      return true;                      
   }
   
}
      		