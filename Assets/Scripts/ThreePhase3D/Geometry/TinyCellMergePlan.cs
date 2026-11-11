using System;
using System.Collections.Generic;
using PhysicsSim.ThreePhase3D.Core;

namespace PhysicsSim.ThreePhase3D.Geometry
{
    // A control-volume grouping plan, not a geometry/inventory edit. Future field
    // operators must sum member inventories and external faces, cancelling internal
    // faces exactly. Isolated tiny components are rejected rather than deleted.
    // Physics-ID: GEOMETRY-TINY-CELL-CONTROL-VOLUMES
    // Implements: R009 R055 R058
    // Model: conservative grouping across the largest accessible shared face
    // Sources: project finite-volume accounting contract
    // Source-Locators: S03-Grid-Geometry.md tiny-cell treatment
    // Inputs-SI: cut-cell fluid volumes [m3], apertures [m2], threshold fraction [1]
    // Outputs-SI: immutable group map [1], aggregate control-volume size [m3]
    // State-Ownership: new grouping plan; source geometry and inventories unchanged
    // Conservation: sum extensive inventories, cancel internal faces; never delete tiny pockets
    // Discretisation: disjoint-set grouping of face-connected cells
    // Validity: finite threshold in (0,1); isolated tiny components rejected explicitly
    // Validation: V06 V35
    // Evidence: PENDING
    // Status: IMPLEMENTED_UNVERIFIED
    // Limitations: grouping plan only; field operators must apply group flux cancellation in S04
    public sealed class TinyCellMergePlan
    {
        private readonly int[] _Group;
        private readonly double[] _GroupFluidVolumeM3;
        public readonly IReadOnlyList<int> Group;
        public readonly IReadOnlyList<double> GroupFluidVolumeM3;
        public readonly int MergedCellCount;
        private readonly bool[] physical;
        public TinyCellMergePlan(CutCellGeometry geometry,double fraction=.05)
        {
            if(!SiUnits.IsFinite(fraction)||fraction<=0||fraction>=1)throw new ArgumentException("Tiny-cell threshold must lie strictly between 0 and 1.");
            var g=geometry.Grid;_Group=new int[g.Cells.Count];_GroupFluidVolumeM3=new double[_Group.Length];physical=new bool[_Group.Length];for(int q=0;q<_Group.Length;q++){_Group[q]=q;g.Cells.Coordinates(q,out int i,out int j,out int k);physical[q]=g.Cells.Interior(i,j,k);}
            int merged=0;
            for(int k=0;k<g.Cells.Z;k++)for(int j=0;j<g.Cells.Y;j++)for(int i=0;i<g.Cells.X;i++)
            {
                int q=g.Cells.Index(i,j,k);double volume=geometry.FluidVolumeM3[q];if(volume<=0||volume>=fraction*g.CellVolume)continue;
                int target=-1;double greatest=0;
                for(int axis=0;axis<3;axis++)foreach(int sign in new[]{-1,1})
                {
                    int ni=i+(axis==0?sign:0),nj=j+(axis==1?sign:0),nk=k+(axis==2?sign:0);if(!g.Cells.Interior(ni,nj,nk))continue;int other=g.Cells.Index(ni,nj,nk);if(geometry.FluidVolumeM3[other]<=0)continue;
                    int fi=i+(axis==0&&sign>0?1:0),fj=j+(axis==1&&sign>0?1:0),fk=k+(axis==2&&sign>0?1:0);double area=geometry.Areas(axis)[g.Shape((GridLocation)(axis+1)).Index(fi,fj,fk)];
                    if(area>greatest){greatest=area;target=other;}
                }
                if(target<0)throw new NotSupportedException("An isolated tiny fluid component requires an explicit component treatment; no volume was discarded.");
                int a=Root(q),b=Root(target);if(a!=b){_Group[a]=b;merged++;}
            }
            for(int q=0;q<_Group.Length;q++){_Group[q]=Root(q);_GroupFluidVolumeM3[_Group[q]]+=geometry.FluidVolumeM3[q];}
            // Two or more tiny cells can form a disconnected small component.
            // Continue agglomerating groups, or reject the whole component.
            for(int root=0;root<_Group.Length;root++)
            {
                int current=Root(root);if(current!=root)continue;
                while(_GroupFluidVolumeM3[current]>0&&_GroupFluidVolumeM3[current]<fraction*g.CellVolume)
                {
                    int target=-1;double greatest=0;
                    for(int q=0;q<_Group.Length;q++)if(physical[q]&&Root(q)==current)
                    {
                        g.Cells.Coordinates(q,out int i,out int j,out int k);
                        for(int axis=0;axis<3;axis++)foreach(int sign in new[]{-1,1})
                        {int ni=i+(axis==0?sign:0),nj=j+(axis==1?sign:0),nk=k+(axis==2?sign:0);if(!g.Cells.Interior(ni,nj,nk))continue;int other=g.Cells.Index(ni,nj,nk);if(geometry.FluidVolumeM3[other]<=0||Root(other)==current)continue;int fi=i+(axis==0&&sign>0?1:0),fj=j+(axis==1&&sign>0?1:0),fk=k+(axis==2&&sign>0?1:0);double area=geometry.Areas(axis)[g.Shape((GridLocation)(axis+1)).Index(fi,fj,fk)];if(area>greatest){greatest=area;target=Root(other);}}
                    }
                    if(target<0)throw new NotSupportedException("An isolated tiny connected component remains below the grouping threshold; no volume was discarded.");
                    _Group[current]=target;_GroupFluidVolumeM3[target]+=_GroupFluidVolumeM3[current];_GroupFluidVolumeM3[current]=0;current=target;merged++;
                }
            }
            for(int q=0;q<_Group.Length;q++)_Group[q]=Root(q);MergedCellCount=merged;Group=Array.AsReadOnly(_Group);GroupFluidVolumeM3=Array.AsReadOnly(_GroupFluidVolumeM3);
        }
        private int Root(int q){int r=q;while(_Group[r]!=r)r=_Group[r];while(_Group[q]!=q){int next=_Group[q];_Group[q]=r;q=next;}return r;}
        public double[] AggregateInventory(double[] extensive)
        {if(extensive==null||extensive.Length!=_Group.Length)throw new ArgumentException("Inventory shape mismatch.");var grouped=new double[_Group.Length];for(int i=0;i<_Group.Length;i++){if(!SiUnits.IsFinite(extensive[i])||(!physical[i]&&extensive[i]!=0))throw new ArgumentException("Nonfinite inventory or owned inventory in a ghost cell.");grouped[_Group[i]]+=extensive[i];}return grouped;}
    }
}
