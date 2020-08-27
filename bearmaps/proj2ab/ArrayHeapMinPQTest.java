package bearmaps.proj2ab;

import org.junit.Test;
import static org.junit.Assert.assertEquals;
import edu.princeton.cs.algs4.Stopwatch;

public class ArrayHeapMinPQTest {
    @Test
    public void testAdd(){
        Stopwatch sw = new Stopwatch();
        ArrayHeapMinPQ a = new ArrayHeapMinPQ();
        for(int i = 0 ; i < 5000;i++){
            a.add(i,i);
        }
        //assertEquals(6,a.size());
        System.out.println("Total time elapsed: " + sw.elapsedTime() +  " seconds.");

    }
    @Test
    public void testAdd2(){
        Stopwatch sw = new Stopwatch();
        ArrayHeapMinPQ a = new ArrayHeapMinPQ();
        a.add(4,4);
        a.add(3,3);
        a.add(6,6);
        a.add(2,2);
        a.add(5,5);
        a.add(1,1);
        a.add(7,7);
        a.add(8,8);
        a.add(10,10);
        a.add(11,11);
        a.add(24,24);
        a.add(15,15);
        a.print();


        //assertEquals(6,a.size());
        System.out.println("Total time elapsed: " + sw.elapsedTime() +  " seconds.");

    }
    @Test
    public void testSmallest(){
        ArrayHeapMinPQ a = new ArrayHeapMinPQ();
        //assertEquals("t1",a.getSmallest());
        Stopwatch sw = new Stopwatch();
        for(int i = 0 ; i < 5000;i++){
            a.add(i,i);
        }
        a.getSmallest();
        //assertEquals(6,a.size());
        System.out.println("Total time elapsed: " + sw.elapsedTime() +  " seconds.");
    }
    @Test
    public void testLeftRight(){
        ArrayHeapMinPQ a = new ArrayHeapMinPQ();
        a.add("t1",1);
        a.add("t2",2);
        a.add("t3",3);
        assertEquals(1,a.leftChild(0));
        assertEquals(2,a.rightChild(0));

    }

    @Test
    public void testRemoveSmallest(){
        ArrayHeapMinPQ a = new ArrayHeapMinPQ();
//        assertEquals("t1",a.removeSmallest());
//        assertEquals(6,a.size());
        Stopwatch sw = new Stopwatch();
        for(int i = 0 ; i < 5000;i++){
            a.add(i,i);
        }
        for(int i = 0 ; i < a.size();i++){
            a.removeSmallest();
        }
        System.out.println("Total time elapsed: " + sw.elapsedTime() +  " seconds.");

    }
    @Test
    public void testChangePriority(){
        ArrayHeapMinPQ a = new ArrayHeapMinPQ();
        a.add(1,1);
        a.add(2,2);
        a.add(3,3);

        assertEquals(1,a.getSmallest());
        a.changePriority(1,5);
        assertEquals(2,a.getSmallest());
        a.changePriority(2,10);
        assertEquals(3,a.getSmallest());

    }

    @Test
    public void testPrint(){
        ArrayHeapMinPQ a = new ArrayHeapMinPQ();
        a.add(1,1);
        a.add(2,2);
        a.add(3,3);
        a.add(4,4);
        a.add(5,5);
        a.add(6,6);
        a.add(9,7);

        a.print();
    }

}