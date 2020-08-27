package bearmaps.proj2ab;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;

public class ArrayHeapMinPQ<T> implements ExtrinsicMinPQ<T>{
    private ArrayList<PriorityNode> items; //= new ArrayList<PriorityNode>();
    private Map<T,Integer> hashMap;// = new HashMap<Object,Object>(); //generic map takes in Priority Node and pos
    private int size;
    /* Adds an item with the given priority value. Throws an
     * IllegalArgumentExceptionb if item is already present.
     * You may assume that item is never null. */
    public ArrayHeapMinPQ(){
        items = new ArrayList<PriorityNode>();
        hashMap = new HashMap<T,Integer>();
    }
    @Override
    public void add(T item, double priority){
        if(contains(item)){
            throw new IllegalArgumentException("Cannot add. "+item+ " exists");
        }else{
            items.add(new PriorityNode(item,priority));
            hashMap.put(item,size()-1);
            swim(size()-1);
        }

    }
    /*swimming the min node up*/
    private void swim(int i){
        if(i<=0) return;
        else{
            int indexParent = parent(i);
            PriorityNode currentNode = items.get(i);
            PriorityNode parentNode = items.get(indexParent);
            if(Double.compare(currentNode.getPriority(),
                    parentNode.getPriority())<0){ //currentNode.getPriority()<parentNode.getPriority()
                swap(currentNode,parentNode);
                swim(indexParent);
            }
        }
    }
    private int findMinNode(int parentIndex){
        int minIndex = parentIndex;
        int l = leftChild(parentIndex);
        int r = rightChild(parentIndex);
        //items.get(l).getPriority() < items.get(minIndex).getPriority()
        if(l<=size()-1
                && Double.compare(items.get(l).getPriority(),items.get(minIndex).getPriority())<0){
            minIndex = l;
        }
        //items.get(r).getPriority() < items.get(minIndex).getPriority()
        if(r<=size()-1
                && Double.compare(items.get(r).getPriority(),items.get(minIndex).getPriority())<0){
            minIndex = r;
        }
        return minIndex;
    }
    /*to use in replacing smallest AKA sink-down operation*/
    private void sink(int current){
        int toSink = findMinNode(current);
        if(toSink!=current){
            swap(items.get(current),items.get(toSink));
            sink(toSink);
        }
    }

    /* Returns true if the PQ contains the given item. */
    @Override
    public boolean contains(T item){
        return hashMap.containsKey(item);
    }
    /* Returns the minimum item. Throws NoSuchElementException if the PQ is empty. */
    @Override
    public T getSmallest(){
        if(size()==0) throw new NoSuchElementException("get Smallest cannot return. PQ is empty");
        return items.get(0).getItem();

    }
    /* Removes and returns the minimum item. Throws NoSuchElementException if the PQ is empty. */
    @Override
    public T removeSmallest(){
        if(size()==0){
            throw new NoSuchElementException("Cannot remove Smallest, PQ is empty");
        }
        T ret = getSmallest();
        swap(items.get(0),items.get(size()-1));
        items.remove(size()-1);
        hashMap.remove(ret);
        sink(0);
        return ret;
    }
    /* Returns the number of items in the PQ. */
    @Override
    public int size(){
        return items.size();
    }

    /* Changes the priority of the given item. Throws NoSuchElementException if the item
     * doesn't exist. */
    @Override
    public void changePriority(T item, double priority){
        if(contains(item)){
            int pos = hashMap.get(item);
            PriorityNode current = items.get(pos);
            if(Double.compare(priority,current.getPriority())>0){ //priority > current.getPriority()
                current.setPriority(priority);
                sink(pos);
            }else if(Double.compare(priority,current.getPriority())<0){ //priority < current.getPriority()
                current.setPriority(priority);
                swim(pos);
            }
        }else throw new NoSuchElementException("Cannot change "+ item +" priority.");
    }

    /*return parent pos of children*/
    public int parent(int pos){
        return (pos-1)/2;
    }

    /*return left child pos*/
    public int leftChild(int pos){
        return (pos*2)+1;
    }
    /*return right child pos*/
    public int rightChild(int pos){
        return (pos*2)+2;
    }
    /* swap two items*/
    public void swap(PriorityNode item1,PriorityNode item2){
        if(item1 == null || item2 == null) return;
        int item1Index = hashMap.get(item1.getItem());
        int item2Index = hashMap.get(item2.getItem());
        PriorityNode tmp = item1;
        items.set(item1Index,item2);
        items.set(item2Index,tmp);
        hashMap.replace((T)item2.getItem(),item1Index);
        hashMap.replace((T)item1.getItem(),item2Index);

    }

    public void print() {
//        int toSink = findMinNode(current);
//        if(toSink!=current){
//            System.out.println(items.get(current).getItem());
//            print(toSink);
//        }
        for(int i = 0 ; i< items.size();i++){
            System.out.println(items.get(i).priority);
        }

    }
    private class PriorityNode implements Comparable<PriorityNode> {
        private T item;
        private double priority;

        PriorityNode(T e, double p) {
            this.item = e;
            this.priority = p;
        }

        T getItem() {
            return item;
        }

        double getPriority() {
            return priority;
        }

        void setPriority(double priority) {
            this.priority = priority;
        }

        @Override
        public int compareTo(PriorityNode other) {
            if (other == null) {
                return -1;
            }
            return Double.compare(this.getPriority(), other.getPriority());
        }

        @Override
        @SuppressWarnings("unchecked")
        public boolean equals(Object o) {
            if (o == null || o.getClass() != this.getClass()) {
                return false;
            } else {
                return ((PriorityNode) o).getItem().equals(getItem());
            }
        }

        @Override
        public int hashCode() {
            return item.hashCode();
        }
    }


}
