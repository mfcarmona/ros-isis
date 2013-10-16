package upc.lsi.kemlg.cbr.etools;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;
import java.io.BufferedReader;

/**
 * This class adapts the CaseSolution class to be used in the A-TEAM Project. <br>
 * It adds a method to parse part of the simulation data files the
 * CBR server receives off-line.
 * @see EToolsCBR
 * @see EToolsCase
 * @see CaseSolution
 */
public class EToolsCaseSolution extends CaseSolution 
                               implements java.io.Serializable
{

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  /** 
   *  parametros de OutputP.  
   */ 
    public AttributeValTreeSet outputParams;
 
 

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
   

  /**
   * Creates a case solution with no evaluation associed. <br> 
   * <br>
   * {PRE: true
   * }<br>
   * {POST: an instance of EToolsCaseSolution has been created.
   * }<br>
   */
    public EToolsCaseSolution(Object o)
    {
       super(o);
    }
    

  /**
   * Parses part of the off-line simulation data, getting the solution of a case. <br>
   * No instance of EToolsCaseSolution is needed to call this method, as it is an
   * static method.
   * {PRE: in is a Java Reader stream connected to a file containing the off-line simulation data
   * }<br>
   * {POST: a instance of EToolsCaseSolution is created with the information related to the case solution. 
   * }<br>
   * <br>
   * @param in the source of cases to be parsed 
   * @return EToolsCaseSolution the case solution obtained.
   */
    public static EToolsCaseSolution importFromData(AttributeTable attrTab, 
                                                   String[] arrSol,
                                                   int numVars,
                                                   BufferedReader in)
    {

       //File sol_file;
       //int begin, end;
       //CaseSolution sol;
	   AttributeDef auxDef;
       AttributeVal auxAtr;
       Modality auxMod;
       int auxType;
	   AttributeValTreeSet desc = new AttributeValTreeSet(attrTab,null);
	   String line = "";
	   float[] arrVal = null;
    	 
       //WARNING!! por ahora solo salta   
	    //OUTPUTG..ENDOUTPUTG, SOLUTION..ENDSOLUTION
	    
       try
       {
	   	  //buscamos OUTPUTP..ENDOUTPUTP,
	       line = in.readLine();
          while ((line != null) && (!line.startsWith("OUTPUTP")))
 	       {
		 	    line = in.readLine();
				//System.out.println(line);
 	       }
		    if (line == null)
		    {
		       throw new java.io.IOException("INCORRECT format of the data. ENDSOLUTION not found.");
          }
		  
		  arrVal = EToolsCaseDescription.parseValues(in, numVars);
		  
		  while ((line != null) && (!line.startsWith("ENDSOLUTION")))
 	       {
		 	    line = in.readLine();
				//System.out.println(line);
 	       }
		    if (line == null)
		    {
		       throw new java.io.IOException("INCORRECT format of the data. ENDSOLUTION not found.");
          }
       }   
       catch (java.io.IOException e)
       {
          e.printStackTrace(); 
       }
	   
       for (int i=0; i<numVars; i++)
       {
          if (attrTab.containsKey(arrSol[i]))
          {
             //System.out.println("tabla contiene " + arrDesc[i]);
             
             auxDef = (AttributeDef) attrTab.get(arrSol[i]);
             //System.out.println ("tipus de l'atribut: " + auxDef.getStrAttrType());
             auxType = auxDef.getAttrType();
             if (auxType == AttributeDef.LINEAR)
             {
			 	if (auxDef.isCyclic())
				{
					auxAtr = new GradeAttribute(auxDef,arrVal[i]);
				}	
				else
				{		
                    auxAtr = new LinearAttribute(auxDef,arrVal[i]);
				} 
             }
             else //es CATEGORICAL
             {
               auxMod = auxDef.getModality((int) (arrVal[i] - 1)); 
               //nota: aquí hay un parche, restamos 1 al arrVal[i] porque en
               // la tabla AttributeDef referenciamos el numero de modalidad
               // de los valores categoricos en el intervalo 0..N-1,
               // en vez de como lo hace RTXPS, de 1..N
               auxAtr = new CategoricalAttribute(auxDef,auxMod.getQualMod());
             }
             desc.add(auxAtr);
          }
          else
          {
             System.out.println("WARNING: attribute table does NOT contain the attribute " + arrSol[i]);
          }
       }   
        
		EToolsCaseSolution solution = new EToolsCaseSolution(desc);
		solution.outputParams = desc;	  
    	
       return solution; 
    } 
   
    /**
   * Parses part of the off-line simulation data, getting the solution of a case. <br>
   * No instance of EToolsCaseSolution is needed to call this method, as it is an
   * static method.
   * {PRE: in is a Java Reader stream connected to a file containing the off-line simulation data
   * }<br>
   * {POST: a instance of EToolsCaseSolution is created with the information related to the case solution. 
   * }<br>
   * <br>
   * @param in the source of cases to be parsed 
   * @return EToolsCaseSolution the case solution obtained.
   */
    public static EToolsCaseSolution importFromData(AttributeTable attrTab, 
                                                   String[] arrSol,
                                                   int numVars,
                                                   BufferedReader in,
                                                   boolean debug)
    {

       //File sol_file;
       //int begin, end;
       //CaseSolution sol;
	   AttributeDef auxDef;
       AttributeVal auxAtr;
       Modality auxMod;
       int auxType;
	   AttributeValTreeSet desc = new AttributeValTreeSet(attrTab,null);
	   String line = "";
	   float[] arrVal = null;
    	 
       //WARNING!! por ahora solo salta   
	    //OUTPUTG..ENDOUTPUTG, SOLUTION..ENDSOLUTION
	    
       try
       {
	   	  //buscamos OUTPUTP..ENDOUTPUTP,
	       line = in.readLine();
          while ((line != null) && (!line.startsWith("OUTPUTP")))
 	       {
		 	    line = in.readLine();
				//System.out.println(line);
 	       }
		    if (line == null)
		    {
		       throw new java.io.IOException("INCORRECT format of the data. ENDSOLUTION not found.");
          }
		  
		  arrVal = EToolsCaseDescription.parseValues(in, numVars,debug);
		  
		  while ((line != null) && (!line.startsWith("ENDSOLUTION")))
 	       {
		 	    line = in.readLine();
				//System.out.println(line);
 	       }
		    if (line == null)
		    {
		       throw new java.io.IOException("INCORRECT format of the data. ENDSOLUTION not found.");
          }
       }   
       catch (java.io.IOException e)
       {
          e.printStackTrace(); 
       }
	   
       for (int i=0; i<numVars; i++)
       {
          if (attrTab.containsKey(arrSol[i]))
          {
             //System.out.println("tabla contiene " + arrDesc[i]);
             
             auxDef = (AttributeDef) attrTab.get(arrSol[i]);
             //System.out.println ("tipus de l'atribut: " + auxDef.getStrAttrType());
             auxType = auxDef.getAttrType();
             if (auxType == AttributeDef.LINEAR)
             {
			 	if (auxDef.isCyclic())
				{
					auxAtr = new GradeAttribute(auxDef,arrVal[i]);
				}	
				else
				{		
                    auxAtr = new LinearAttribute(auxDef,arrVal[i]);
				} 
             }
             else //es CATEGORICAL
             {
               auxMod = auxDef.getModality((int) (arrVal[i] - 1)); 
               //nota: aquí hay un parche, restamos 1 al arrVal[i] porque en
               // la tabla AttributeDef referenciamos el numero de modalidad
               // de los valores categoricos en el intervalo 0..N-1,
               // en vez de como lo hace RTXPS, de 1..N
               auxAtr = new CategoricalAttribute(auxDef,auxMod.getQualMod());
             }
             desc.add(auxAtr);
          }
          else
          {
             System.out.println("WARNING: attribute table does NOT contain the attribute " + arrSol[i]);
          }
       }   
        
		EToolsCaseSolution solution = new EToolsCaseSolution(desc);
		solution.outputParams = desc;	  
    	
       return solution; 
    }  

  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of EToolsCaseSolution 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public String toString() //provisional
    {
       return getSolution().toString();
    }                                             
}