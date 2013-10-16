package upc.lsi.kemlg.general;

import java.io.*;
import java.util.Vector;
import java.util.Iterator;

/**
 * This class assigns and deassigns number identifiers. <br>
 * It manages the identifiers assigned and deassigned in a way that:
 * <ul><li>it gives an identifier that is not currntly assigned</li>
 *     <li>when a identifier is de-assigned, it can be assigned again
 *     </li>
 * </ul>
 * The second property makes that the system using this class won't
 * run out of numbers by means of assigning and deassigning them (as 
 * it could happen when a simple counter is used), as the deassigned
 * identifiers are used again.
 * The class takes care of the maximum number assigned and the "holes",
 * thiat is, the deassigned numbers from 0 to the maximum number 
 * currently assigned. The management of holes is made through a list
 * (a java Vector) where deassigned identifiers are stored as Strings.
 * When assigning a indentifier, if there are holes then an identifier
 * from a hole is given instead of increasing by one the maximum
 * assigned number. <br>
 * This class is syncronized for its use with threads.
 */

public class IdGenerator implements Serializable 
{
  
  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  
  /**
   * Initial size of the holes vector.
   */
   private static final int LONGITUD = 100;

  /**
   * Increment of the size of the holes vector.
   */
   private static final double INCREMENTO_VECTOR = 100;
   
  /**
   * The maximum identifier value currently assigned.
   */  
   private int maxId; //el Identificador mayor que se ha asignado.
     
  /**
   * The list of holes (deassigned identifiers between 0 and maxId).   
   */
   private Vector huecosId; //Llamaremos a partir de ahora a estos 
        	                  //identificadores desasignados Huecos 
                            //(en el espacio de asignaciones).
            

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  
  /**
   * Creates an empty IdGenerator(). <br>
   * <br>
   * {PRE: true
   * }<br>
   * {POST: an identifier generator has been created.
   * }<br>
   */
   public IdGenerator ()
   {
   	 maxId = 0;
   	 huecosId = new Vector(LONGITUD);
   }
	

  /**
   * Assigns a identifier. <br>
   * <br>
   * {PRE: x is an instance of IdGenerator
   * }<br>
   * {POST: returns a non-previously assigned identifier, and x updates
   *        its internal structure to remember it.
   * }<br>
   * @return int the identifier to assign
   */
   public synchronized int assignId ()
   {
   	  int nuevoId;
   	  
  	     if (huecosId.isEmpty())  //no hay huecos por asignar
        {
           maxId++;
           nuevoId = maxId;
        }
        else //se devuelve uno de los huecos.
        {
        	  nuevoId = Integer.parseInt((String) huecosId.get(0));
        	  huecosId.removeElementAt(0);
        }
   	
   	  return nuevoId;
   }
	

  /**
   * Deassigns the given identifier. <br>
   * <br>
   * {PRE: x is an instance of IdGenerator, and
   *       id is a identifier previously assigned
   * }<br>
   * {POST: the identifier id is marked as unassigned form now on,
   *        and x updates its internal structure to remember it.
   * }<br>
   * @param int the identifier to deassign
   */
   public synchronized void deassignId (int id)
   {
   	if (id == maxId) 
      {
   	   maxId--;
      	while (huecosId.contains(Integer.toString(maxId)))
         {
         	huecosId.remove(Integer.toString(maxId));
         	maxId--;
         }
   	   // comprobar si los numeros anteriores son huecos **********
      }	
   	else if ((id > maxId) || (id < 1))
   	   System.err.println ("Error: Este Id no existe");
      else //hay que generar un hueco.
      {
      	// comprobamos que el número no sea ya un hueco.
      	if (huecosId.contains(Integer.toString(id)))
      		System.err.println ("Error: Este Id no esta asignado a nadie");
      	else //el id no es aun un hueco
      		huecosId.addElement(Integer.toString(id));
      	   // comprobar si hay capacidad en el vector, sino incrementarla *****
      }
   }


  /**
   * Assigns all the number identifiers inside a given Vector. It is
   * useful to initialise the internal state of a IdGenerator with the
   * identifiers already assigned.<br>
   * <br>
   * {PRE: x is an instance of IdGenerator, 
   *       ids is a java Vector which each element is a integer number 
               in String format, and there are no repetition of numbers 
               in ids. 
   * }<br>
   * {POST: returns a non-previously assigned identifier, and x updates
   *        its internal structure to remember it.
   * }<br>
   * @return int the identifier to assign
   */
   public void assignCurrentIds(Vector ids)
   {
      int contador_ids=1;
      int elem_actual;

      for (Iterator i=ids.iterator(); i.hasNext(); )
      {
         elem_actual= Integer.parseInt((String)i.next());

         while (contador_ids < elem_actual)
         {
            huecosId.addElement(Integer.toString(contador_ids));
            contador_ids++;
         }
 
         // {contador_ids == elem_actual)
         contador_ids++;
      }

      maxId = contador_ids - 1;
   }
}
      		