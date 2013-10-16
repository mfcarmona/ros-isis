package upc.lsi.kemlg.cbr.etools;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;
import java.io.*;


/**
 * This class adapts the AttributeValTree class (the kind of case description
 * chosen) to be used in the A-TEAM Project. <br>
 * It adds a method to parse part of the simulation data files the
 * CBR server receives off-line.
 * @see EToolsCBR
 * @see AttributeValTreeSet
 * @see EToolsCase
 */
public class EToolsCaseDescription extends AttributeValTreeSet 
                                  implements java.io.Serializable
{
	  

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
   

  /**
   * Creates an empty EToolsCaseDescription, with the given attribute table and distance function. <br> 
   * <br>
   * {PRE: attrTab is instance of AttributeTable
   *       dist is an instance of ComposedDistance
   * }<br>
   * {POST: an instance of EToolsCaseDescription has been created.
   * }<br>
   */
    public EToolsCaseDescription(AttributeTable attrTab, ComposedDistance dist)
    {
       super(attrTab, dist);
    }
    

  /**
   * Parses part of the off-line simulation data, getting the description of a case. <br>
   * The information to process is passed through arrays.
   * No instance of EToolsCaseDescription is needed to call this method, as it is an
   * static method.
   * {PRE: attrTab is the attribute table, the one that has the attribute definitions;
           dist is the distance function that the attributes will use to do
                comparison among themselves;
           arrDesc is an array with the names of the attributes in the
                   order their values will appear
           numVars is the number of attributes that form part of the description
   * }<br>
   * {POST: if the name of all the attributes in arrDesc joined with the values in arrVal 
            match the definition in attrTab then returns an instance of
            EToolsCaseDescription with all the information associed. 
   * }<br>
   * <br>
   * @param attrTab the attribute table that contains the attribute definitions
   * @param dist the distance function
   * @param arrDesc the array of attribute names
   * @param arrVal the array of attribute values
   * @param numVars the number of attributes that form the case description
   * @param in the source of cases to be parsed 
   * @return EToolsCaseDescription the case description obtained.
   */
    public static EToolsCaseDescription importFromData(AttributeTable attrTab, 
                                           ComposedDistance dist,
                                           String[] arrDesc,
                                           int numVars,
                                           BufferedReader in)
    {
       AttributeDef auxDef;
       AttributeVal auxAtr;
       Modality auxMod;
       int auxType;
       EToolsCaseDescription desc = new EToolsCaseDescription(attrTab,dist);
       float[] arrVal = null;
	   String line;
       

       //1. Obtenemos la descripción del caso  
       try
       {
	      line = in.readLine();
          while ((line != null) && (!line.startsWith("INPUT")))
 	      {
		     line = in.readLine();
			 //System.out.println("line = " + line);
 	      }
		  if (line == null)
		  {
		     throw new java.io.IOException("INCORRECT format of the data. INPUT not found.");
          }

          arrVal = EToolsCaseDescription.parseValues(in, numVars);

       //2. Obtenemos los trainee

          //WARNING!!: por ahora solo leeremos los parametros de entrada...

          //saltamos hasta ENDTRAINEE
		  //WARNING! esto solo se ha de hacer si es CaseSPECIFIED!!!
          while ((line != null) && (!line.startsWith("ENDTRAINEE")))
 	      {
		     line = in.readLine();
 	         //System.out.println("line = " + line);
 	      }
		  if (line == null)
		  {
		     throw new java.io.IOException("INCORRECT format of the data. ENDTRAINEE not found.");
          }

          
       }   
       catch (IOException e)
       {
          e.printStackTrace(); 
       }
       
    
	   
       
       
       //3. Creamos el CaseDescription
       
       for (int i=0; i<numVars; i++)
       {
          if (attrTab.containsKey(arrDesc[i]))
          {
             //System.out.println("tabla contiene " + arrDesc[i]);
             
             auxDef = (AttributeDef) attrTab.get(arrDesc[i]);
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
             System.out.println("WARNING: attribute table does NOT contain the attribute " + arrDesc[i]);
          }
       }   
       
       return desc;
    }                                              
    
    /**
   * Parses part of the off-line simulation data, getting the description of a case. <br>
   * The information to process is passed through arrays.
   * No instance of EToolsCaseDescription is needed to call this method, as it is an
   * static method.
   * {PRE: attrTab is the attribute table, the one that has the attribute definitions;
           dist is the distance function that the attributes will use to do
                comparison among themselves;
           arrDesc is an array with the names of the attributes in the
                   order their values will appear
           numVars is the number of attributes that form part of the description
   * }<br>
   * {POST: if the name of all the attributes in arrDesc joined with the values in arrVal 
            match the definition in attrTab then returns an instance of
            EToolsCaseDescription with all the information associed. 
   * }<br>
   * <br>
   * @param attrTab the attribute table that contains the attribute definitions
   * @param dist the distance function
   * @param arrDesc the array of attribute names
   * @param arrVal the array of attribute values
   * @param numVars the number of attributes that form the case description
   * @param in the source of cases to be parsed 
   * @return EToolsCaseDescription the case description obtained.
   */
    public static EToolsCaseDescription importFromData(AttributeTable attrTab, 
                                           ComposedDistance dist,
                                           String[] arrDesc,
                                           int numVars,
                                           BufferedReader in,
                                           boolean debug)
    {
       AttributeDef auxDef;
       AttributeVal auxAtr;
       Modality auxMod;
       int auxType;
       EToolsCaseDescription desc = new EToolsCaseDescription(attrTab,dist);
       float[] arrVal = null;
	   String line;
       

       //1. Obtenemos la descripción del caso  
       try
       {
	      line = in.readLine();
          while ((line != null) && (!line.startsWith("INPUT")))
 	      {
		     line = in.readLine();
			 //System.out.println("line = " + line);
 	      }
		  if (line == null)
		  {
		     throw new java.io.IOException("INCORRECT format of the data. INPUT not found.");
          }

          arrVal = EToolsCaseDescription.parseValues(in, numVars,debug);

       //2. Obtenemos los trainee

          //WARNING!!: por ahora solo leeremos los parametros de entrada...

          //saltamos hasta ENDTRAINEE
		  //WARNING! esto solo se ha de hacer si es CaseSPECIFIED!!!
          while ((line != null) && (!line.startsWith("ENDTRAINEE")))
 	      {
		     line = in.readLine();
 	         //System.out.println("line = " + line);
 	      }
		  if (line == null)
		  {
		     throw new java.io.IOException("INCORRECT format of the data. ENDTRAINEE not found.");
          }

          
       }   
       catch (IOException e)
       {
          e.printStackTrace(); 
       }
       
    
	   
       
       
       //3. Creamos el CaseDescription
       
       for (int i=0; i<numVars; i++)
       {
          if (attrTab.containsKey(arrDesc[i]))
          {
             //System.out.println("tabla contiene " + arrDesc[i]);
             
             auxDef = (AttributeDef) attrTab.get(arrDesc[i]);
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
             System.out.println("WARNING: attribute table does NOT contain the attribute " + arrDesc[i]);
          }
       }   
       
       return desc;
    } 
	
	 /**
   * Parses a case description of a case, passed through arrays.
   * No instance of EToolsCaseDescription is needed to call this method, as it is an
   * static method.
   * {PRE: attrTab is the attribute table, the one that has the attribute definitions;
           dist is the distance function that the attributes will use to do
                comparison among themselves;
           arrDesc is an array with the names of the attributes in the
                   order their values will appear
           numVars is the number of attributes that form part of the description
   * }<br>
   * {POST: if the name of all the attributes in arrDesc joined with the values in arrVal 
            match the definition in attrTab then returns an instance of
            EToolsCaseDescription with all the information associed. 
   * }<br>
   * <br>
   * @param attrTab the attribute table that contains the attribute definitions
   * @param dist the distance function
   * @param arrDesc the array of attribute names
   * @param arrVal the array of attribute values
   * @param numVars the number of attributes that form the case description
   * @param in the source of cases to be parsed 
   * @return EToolsCaseDescription the case description obtained.
   */
    public static EToolsCaseDescription parseFromArray(AttributeTable attrTab, 
                                           ComposedDistance dist,
                                           String[] arrDesc,
                                           float[] arrVal,
                                           int numVars)
    {
       AttributeDef auxDef;
       AttributeVal auxAtr;
       Modality auxMod;
       int auxType;
       EToolsCaseDescription desc = new EToolsCaseDescription(attrTab,dist);
       
       //obtener la descripción del caso
       
       for (int i=0; i<numVars; i++)
       {
          if (attrTab.containsKey(arrDesc[i]))
          {
             //System.out.println("tabla contiene " + arrDesc[i]);
             
             auxDef = (AttributeDef) attrTab.get(arrDesc[i]);
             //System.out.println ("tipus de l'atribut: " + auxDef.getStrAttrType());
             auxType = auxDef.getAttrType();
             if (auxType == AttributeDef.LINEAR)
             {
               auxAtr = new LinearAttribute(auxDef,arrVal[i]);
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
             //System.out.println("WARNING: attribute table does NOT contain the attribute " + arrDesc[i]);
          }
       }   
       
       return desc;
    }                      
	
  //====||===============================================================
  //===\||/================ metodos auxiliares ==========================
  //====\/===============================================================

    public static float[] parseValues(BufferedReader in, int numVars)
	        throws java.io.IOException
    {
       float[] arrayVal; 
	   String line;

       StreamTokenizer st;
       int tokenType;
	   float tokenNum =-1.0f;	
	   String tokenStr = "";
	   
	   st =  new StreamTokenizer(in);
       st.eolIsSignificant(false);    

	    arrayVal = new float[numVars];
	   
	   st.nextToken();
       for (int i=0;i<numVars;i++)
	    {
	       if (st.ttype == StreamTokenizer.TT_NUMBER)
	       {
	           arrayVal[i] = (float)st.nval;
//	           //System.out.println("#" + arrayVal[i] + "#");        
	       }   
	       else 
	       {
	           //System.out.println("<??>" + st.ttype );
	           throw new java.io.IOException("INCORRECT format of the data. Found " + st.ttype + " token instead.");
	       }
	       st.nextToken();
	    }
	    line = "#EToolsCaseDescription: Valores = #";
	    for (int i=0;i<numVars;i++)
	    {
	       line += "" + arrayVal[i] + "#";
	    }
	    System.out.println(line);
	    
	    return arrayVal;
	 }	
	   
	 public static float[] parseValues(BufferedReader in, int numVars, boolean debug)
	        throws java.io.IOException
    {
       float[] arrayVal; 
	   String line;

       StreamTokenizer st;
       int tokenType;
	   float tokenNum =-1.0f;	
	   String tokenStr = "";
	   
	   st =  new StreamTokenizer(in);
       st.eolIsSignificant(false);    

	    arrayVal = new float[numVars];
	   
	   st.nextToken();
       for (int i=0;i<numVars;i++)
	    {
	       if (st.ttype == StreamTokenizer.TT_NUMBER)
	       {
	           arrayVal[i] = (float)st.nval;
//	           //System.out.println("#" + arrayVal[i] + "#");        
	       }   
	       else 
	       {
	           //System.out.println("<??>" + st.ttype );
	           throw new java.io.IOException("INCORRECT format of the data. Found " + st.ttype + " token instead.");
	       }
	       st.nextToken();
	    }
	    line = "_EToolsCaseDescription: Valores = #";
	    for (int i=0;i<numVars;i++)
	    {
	       line += "" + arrayVal[i] + "#";
	    }
	    if (debug)
	    	 System.out.println(line);
	    
	    return arrayVal;
	 }	
	   
      
}